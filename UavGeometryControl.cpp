/*
 * UavGeometryControl.cpp
 *
 *  Created on: Jul 23, 2018
 *      Author: lmark
 */

#include "UavGeometryControl.h"

const double G = 9.81;

// UAV constants
const double UAV_MASS = 2.1;
const double ARM_LENGTH = 0.314;
const double MOMENT_CONSTANT = 0.016;
const double MOTOR_CONSTANT = 8.54858e-06;

// ROTOR constants
const double ROTOR_MASS = 0.01;
const double ROTOR_VELOCITY_SLOWDOWN_SIM = 15;
const double ROTOR_RADIUS = 0.1524;
const double MIN_ROTOR_VELOCITY = 0;
const double MAX_ROTOR_VELOCITY = 1475;
Matrix<double, 3, 3> INERTIA;
const double D =  ARM_LENGTH + ROTOR_RADIUS / 2;
const double MAXIMUM_MOMENT =
		MAX_ROTOR_VELOCITY * MAX_ROTOR_VELOCITY * MOTOR_CONSTANT // MAX FORCE
		* D;

// Moving mass constants
const double MM_MASS = 0.208;
const double MM_FORCE = MM_MASS * G;

// Transform matrix with roll / pitch corrections
Matrix<double, 4, 4> THRUST_TRANSFORM_FULL;

// Transform matrix without roll / pitch corrections
Matrix<double, 4, 4> THRUST_TRANSFORM_YAW;

const Matrix<double, 3, 1> E3;
Matrix<double, 3, 3> EYE3;
Matrix<double, 4, 4> EYE4;

// Define control modes
const int POSITION_CONTROL = 1;
const int ATTITUDE_CONTROL = 2;
const int VELOCITY_CONTROL = 3;

UavGeometryControl::UavGeometryControl() :
	geometry_control_node_handle_(geometry_control_app_state_),
	params_(),
	geometry_control_loop_rate_(params_.controller_sample_rate),

	geometry_control_task_(-1),
	geometry_control_task_should_exit_(false),

	armed_flag_(attitude_controller_reference_s::DISARMED),
	roll_control_flag_previous_(attitude_controller_reference_s::OFF),
	roll_control_flag_(attitude_controller_reference_s::OFF),
	pitch_control_flag_previous_(attitude_controller_reference_s::OFF),
	pitch_control_flag_(attitude_controller_reference_s::OFF),
	yaw_control_flag_previous_(attitude_controller_reference_s::OFF),
	yaw_control_flag_(attitude_controller_reference_s::OFF),
	yaw_rate_control_flag_previous_(attitude_controller_reference_s::OFF),
	yaw_rate_control_flag_(attitude_controller_reference_s::OFF),

	current_control_mode_(ATTITUDE_CONTROL),
	enable_mass_control_(false)
{
	// Initialize inertia matrix
	INERTIA.setZero();
	INERTIA(0, 0) = 0.0826944;
	INERTIA(1, 1) = 0.0826944;
	INERTIA(2, 2) = 0.0104;

	Matrix<double, 3, 3> rotor_inertia;
	rotor_inertia.setZero();
	rotor_inertia(0, 0) = 1/12 * ROTOR_MASS
			* (0.031 * 0.031 + 0.005 * 0.005)
			* ROTOR_VELOCITY_SLOWDOWN_SIM;
	rotor_inertia(1, 1) = 1/12 * ROTOR_MASS
			* (4 * ROTOR_RADIUS * ROTOR_RADIUS + 0.005 * 0.005)
			* ROTOR_VELOCITY_SLOWDOWN_SIM;
	rotor_inertia(2, 2) = 1/12 * ROTOR_MASS
			* (4 * ROTOR_RADIUS * ROTOR_RADIUS + 0.031 * 0.031)
			* ROTOR_VELOCITY_SLOWDOWN_SIM
			+ ROTOR_MASS * ARM_LENGTH * ARM_LENGTH;

	INERTIA = INERTIA + 4 * rotor_inertia;

	// Initialize eye(3) matrix
	EYE3.setZero();
	EYE3(0, 0) = 1;
	EYE3(1, 1) = 1;
	EYE3(2, 2) = 1;

	// Initialize eye(4) matrix
	EYE4.setZero();
	EYE4(0, 0) = 1;
	EYE4(3, 3) = 1;

	// Initialize thrust transform matrix
	SquareMatrix<double, 4> tt_square;
	tt_square.setZero();

	// First row
	tt_square(0, 0) = 1;
	tt_square(0, 1) = 1;
	tt_square(0, 2) = 1;
	tt_square(0, 3) = 1;

	// Second row
	tt_square(1, 1) = D;
	tt_square(1, 3) = - D;

	// Third row
	tt_square(2, 0) = - D;
	tt_square(2, 2) = D;

	// Fourth row
	tt_square(3, 0) = MOMENT_CONSTANT;
	tt_square(3, 1) = - MOMENT_CONSTANT;
	tt_square(3, 2) = MOMENT_CONSTANT;
	tt_square(3, 3) = - MOMENT_CONSTANT;

	// Invert the matrix
	THRUST_TRANSFORM_FULL = inv(tt_square);
	THRUST_TRANSFORM_YAW = THRUST_TRANSFORM_FULL * EYE4;


	// Initialize desired position values
	x_d_.setZero();
	x_d_(2, 0) = 1;
	v_d_.setZero();
	a_d_.setZero();

	// Initial measured position values
	x_mv_.setZero();
	v_mv_.setZero();

	// Initialize desired attitude
	omega_d_.setZero();
	alpha_d_.setZero();
	b1_d_.setZero();
	b1_d_(0,0) = 1;
	R_d_.setZero();

	// Initialize measured
	omega_mv_.setZero();
	R_mv_.setZero();
}

UavGeometryControl::~UavGeometryControl()
{
	if (geometry_control_task_ != -1)
	{
		geometry_control_task_should_exit_ = true;
		int i = 0;
		// Sleep for 0.5s to ensure task exits
		while(geometry_control_task_ != -1)
		{
			usleep(10000);
			if(++i > 50)
			{
				px4_task_delete(geometry_control_task_);
				break;
			}
		}
	}

	uav_geometry_control::instance = nullptr;
}

int UavGeometryControl::start()
{
	ASSERT(geometry_control_task_ == -1);

	// Start attitude control task
	geometry_control_task_ = px4_task_spawn_cmd(
			"mmc_vpc_attitude_control",
			SCHED_DEFAULT,
			SCHED_PRIORITY_MAX - 5,
			1500,
			(px4_main_t)&UavGeometryControl::geometryControlTaskTrampoline,
			nullptr);

	if (geometry_control_task_ < 0)
	{
		warn("Failed to start attitude control task! \n");
		return -errno;
	}

	return OK;
}

void UavGeometryControl::geometryControlTaskTrampoline(int argc, char *argv[])
{
	uav_geometry_control::instance->geometryControlTaskMain();
	PX4_INFO("Returning from geometry control task trampoline! \n");
	return;
}

void UavGeometryControl::geometryControlTaskMain()
{

	// Node frequency is determined by param. 100Hz by default.
	//attitude_control_loop_rate_ = px4::Rate(params_.attitude_sample_rate);

	// Subscribers
	geometry_control_node_handle_.subscribe<px4::px4_parameter_update>(
			&UavGeometryControl::parametersUpdateCallback, this, 1);
	geometry_control_node_handle_.subscribe<px4::px4_vehicle_attitude>(
			&UavGeometryControl::vehicleAttitudeCallback, this, 1);
	geometry_control_node_handle_.subscribe<px4::px4_attitude_controller_reference>(
				&UavGeometryControl::attitudeReferenceCallback, this, 1);

	// Publishers
	moving_mass_setpoint_array_pub_ = geometry_control_node_handle_
			.advertise<px4::px4_moving_mass_setpoint_array>();
	gas_motor_setpoint_array_pub_ = geometry_control_node_handle_
			.advertise<px4::px4_gas_motor_setpoint_array>();
	attitude_controller_status_pub_ = geometry_control_node_handle_
			.advertise<px4::px4_attitude_controller_status>();

	float sample_time = 1.0f/((float)params_.controller_sample_rate);

	/*
	 * b3_d 			- desired thrust vector
	 * M_u 				- control moment
	 * x_des, x_old		- desired position, old position
	 * b1_des, b1_old 	- desired heading, old heading
	 * v_d_old			- old desired velocity
	 */
	Matrix<double, 3, 1> b3_d, M_u, x_old,
						b1_old, x_des, b1_des,
						v_d_old;
	b1_old = b1_d_;
	x_old = x_mv_;

	/*
	 * R_d 		- desired rotation matrix
	 * R_d_old 	- used for matrix differentiation
	 * R_d_dot 	- Desired matrix derivative
	 */
	Matrix<double, 3, 3> R_c, R_c_old, omega_c_old;
	R_c_old = EYE3;
	omega_c_old.setZero();

	double dt = 0.01;
	double f_u;
	Matrix<double, 4, 1> rotor_velocities;
	Matrix<double, 4, 1> thrust_moment_vec;

	while(!geometry_control_task_should_exit_)
	{
		// Sleep and collect callback data
		geometry_control_loop_rate_.sleep();
		geometry_control_node_handle_.spinOnce();

		// Construct current rotation matrix - R
		euler2RotationMatrix(
				(double)attitude_measured_(0),
				(double)attitude_measured_(1),
				(double)attitude_measured_(2),
				R_mv_);

		// Position and heading prefilter
		x_des = x_d_; //x_old + 0.025 * (x_d_ - x_old);
		b1_des = b1_old + 0.05 * (b1_d_ - b1_old);

		// TRAJECTORY TRACKING BLOCK
		trajectoryTracking(
				x_des,		// Input - desired position
				x_old,		// Input - old position
				dt,			// Input - time interval
				b3_d,		// OUTPUT - thrust vector
				f_u);		// OUTPUT - total thrust

		// Update old position
		b1_old = b1_des;
		x_old = x_des;
		v_d_old = v_d_;

		// ATTITUDE TRACKING BLOCK
		attitudeTracking(
				b1_des,			// Input - desired heading
				b3_d,			// Input - desired thrust vector
				dt,				// Input - time interval
				R_c_old,
				omega_c_old,
				M_u);			// OUTPUT - control moments

		// Calculate thrust velocities
		thrust_moment_vec(0, 0) = f_u;
		thrust_moment_vec(0, 0) = M_u(0, 0);
		thrust_moment_vec(0, 0) = M_u(1, 0);
		thrust_moment_vec(0, 0) = M_u(2, 0);

		if (enable_mass_control_)
		{
			// Calculate height and yaw control
			calculateRotorVelocities(
					thrust_moment_vec,
					THRUST_TRANSFORM_YAW,
					rotor_velocities);

			// Roll and pitch control with masses
			double dx = (double)M_u(1, 0) / (2 * MM_FORCE);
			double dy = (double)M_u(0, 0) / (2 * MM_FORCE);

			dx = saturation(dx, -ARM_LENGTH / 2, ARM_LENGTH / 2);
			dy = saturation(dy, -ARM_LENGTH / 2, ARM_LENGTH / 2);

			px4::px4_moving_mass_setpoint_array moving_mass_array;
			moving_mass_array.data().position[moving_mass_setpoint_array_s::FRONT] =
					dx / (ARM_LENGTH / 2);
			moving_mass_array.data().position[moving_mass_setpoint_array_s::RIGHT] =
					- dy / (ARM_LENGTH / 2);
			moving_mass_array.data().position[moving_mass_setpoint_array_s::BACK] =
					- dx / (ARM_LENGTH / 2);
			moving_mass_array.data().position[moving_mass_setpoint_array_s::LEFT] =
					dy / (ARM_LENGTH / 2);
			moving_mass_array.data().timestamp = hrt_absolute_time();
			moving_mass_setpoint_array_pub_->publish(moving_mass_array);

		}
		else
		{
			// Calculate full rotor control
			calculateRotorVelocities(
					thrust_moment_vec,
					THRUST_TRANSFORM_FULL,
					rotor_velocities);
		}

		px4::px4_gas_motor_setpoint_array gas_motor_array;
		gas_motor_array.data().setpoint[gas_motor_setpoint_array_s::FRONT] =
				(double)rotor_velocities(0, 0);
		gas_motor_array.data().setpoint[gas_motor_setpoint_array_s::RIGHT] =
				(double)rotor_velocities(1, 0);
		gas_motor_array.data().setpoint[gas_motor_setpoint_array_s::BACK] =
				(double)rotor_velocities(2, 0);
		gas_motor_array.data().setpoint[gas_motor_setpoint_array_s::LEFT] =
				(double)rotor_velocities(3, 0);
		gas_motor_array.data().timestamp = hrt_absolute_time();
		gas_motor_setpoint_array_pub_->publish(gas_motor_array);
	}
}

void UavGeometryControl::attitudeReferenceCallback(
		const px4::px4_attitude_controller_reference &msg)
{
	attitude_reference_(0)    = msg.data().roll;
	attitude_reference_(1)    = msg.data().pitch;
	attitude_reference_(2)    = msg.data().yaw;
	yaw_rate_feedforward_     = msg.data().yaw_rate;
	rotor_speed_reference_    = msg.data().throttle;

	// Flags
	armed_flag_             = msg.data().armed;
	roll_control_flag_      = msg.data().roll_control;
	pitch_control_flag_     = msg.data().pitch_control;
	yaw_control_flag_       = msg.data().yaw_control;
	yaw_rate_control_flag_  = msg.data().yaw_rate_control;
}

void UavGeometryControl::vehicleAttitudeCallback(
		const px4::px4_vehicle_attitude &msg)
{
	// Set message quaternion to matrix quaternion
	matrix::Quatf q(
			msg.data().q[0],
			msg.data().q[1],
			msg.data().q[2],
			msg.data().q[3]);
	// Quaternion to euler
	matrix::Eulerf e(q);
	//ut_test(isEqual(e1, euler_check));

	// Attitude measured(NED), subtract roll and pitch offset
	attitude_measured_(0) = e(0);
	attitude_measured_(1) = e(1);
	attitude_measured_(2) = e(2);

	//PX4_INFO("%.2f, %.2f, %.2f", (double)attitude_measured_(0),
	//  (double)attitude_measured_(1), (double)attitude_measured_(2));

	// Rates measured(NED)
	omega_mv_(0, 0) = msg.data().rollspeed;
	omega_mv_(1, 0) = msg.data().pitchspeed;
	omega_mv_(2, 0) = msg.data().yawspeed;
}

void UavGeometryControl::parametersUpdateCallback(
		const px4::px4_parameter_update &msg)
{
  PX4_INFO("Parameters updated!");
  params_.update();
}

void UavGeometryControl::calculateRotorVelocities(
		Matrix<double, 4, 1> thrust_moment_vec,
		Matrix<double, 4, 4> transform_matrix,
		Matrix<double, 4, 1>& rotor_velocities)
{
	Matrix<double, 4, 1> rotor_signs;

	// Convert force vector - THRUST_TRANSFORM * thrus_moment_vec ...
	// ...to angular velocity -> fi = MOTOR_CONSTANT * ang_vel_i^2
	rotor_velocities.setZero();
	rotor_velocities = transform_matrix * thrust_moment_vec;
	rotor_signs = vecSign(rotor_velocities);
	rotor_velocities = rotor_velocities.abs();
	rotor_velocities = rotor_velocities / MOTOR_CONSTANT;
	vecSqrt(rotor_velocities);

	rotor_velocities(0, 0) =
			rotor_signs(0, 0) *
			saturation(
					(double)rotor_velocities(0, 0),
					- MAX_ROTOR_VELOCITY,
					MAX_ROTOR_VELOCITY);
	rotor_velocities(1, 0) =
			rotor_signs(1, 0) *
			saturation(
					(double)rotor_velocities(1, 0),
					- MAX_ROTOR_VELOCITY,
					MAX_ROTOR_VELOCITY);
	rotor_velocities(2, 0) =
			rotor_signs(2, 0) *
			saturation(
					(double)rotor_velocities(2, 0),
					- MAX_ROTOR_VELOCITY,
					MAX_ROTOR_VELOCITY);

	rotor_velocities(3, 0) =
			rotor_signs(3, 0) *
			saturation(
					(double)rotor_velocities(3, 0),
					- MAX_ROTOR_VELOCITY,
					MAX_ROTOR_VELOCITY);

}

void UavGeometryControl::calculateDesiredAngularVelAndAcc(
		const Matrix<double, 3, 3> R_c,
		const Matrix<double, 3, 3> R_c_old,
		const Matrix<double, 3, 3> R_mv,
		Matrix<double, 3, 3> &omega_c_old,
		double dt)
{
	Matrix<double, 3, 3> omega_c_skew, alpha_c_skew;

	Matrix<double, 3, 3> R_c_dot = (R_c - R_c_old);
	omega_c_skew = R_c_dot * R_c.transpose();

	// Remap calculated values to desired
	veeOperator(omega_c_skew, omega_d_);
	//veeOperator(alpha_c_skew, alpha_d_);
}

void UavGeometryControl::attitudeTracking(
		const Matrix<double, 3, 1> b1_desired,
		const Matrix<double, 3, 1> b3_desired,
		const double dt,
		Matrix<double, 3, 3> &R_c_old,
		Matrix<double, 3, 3> &omega_c_old,
		Matrix<double, 3, 1> &M_u)
{

	// Attitude errors
	Matrix<double, 3, 1> e_omega, e_R;

	// Auxiliary skew matrices
	Matrix<double, 3, 3> e_R_skew, omega_mv_skew, omega_c_skew,
						 alpha_c_skew;

	if (current_control_mode_ == POSITION_CONTROL)
	{
		//cout << "Attitude: POSITION" << "\n";
		/**
		 * During position control desired rotation, angular velocity
		 * and angular acceleration matrices will be CALCULATED.
		 * R_c, omega_c, alpha_c
		 */
		Matrix<double, 3, 3> R_c;
		Matrix<double, 3, 1> b1_c, b2_c;

		/*
		 * b13_normal - Normal of plane spanned by b3_d and b1_d.
		 *
		 * Note: b1_d will not necessarily lie in the plane with b3_d normal,
		 * it is needed to calculate it's projection to that plane.
		 */
		Matrix<double, 3, 1> b13_normal = cross(b3_desired, b1_desired);

		// Compute b1_c = Proj[b1_d] onto the plane with normal b3_d
		b1_c = - cross(b3_desired, b13_normal) / norm(b13_normal);

		// Construct desired rotation matrix
		b2_c = cross(b3_desired, b1_c);
		b2_c = b2_c / norm(b2_c);
		R_c.setZero();
		R_c << b1_c, b2_c, b3_desired;

		calculateDesiredAngularVelAndAcc(R_c, R_c_old, R_mv_, omega_c_old, dt);

		// Remap calculated to desired
		R_d_ = R_c;


		// Update old R_c
		R_c_old = R_c;
	}
	else if (current_control_mode_ == ATTITUDE_CONTROL)
	{
		//cout << "Attitude: ATTITUDE" << "\n";
		// Do nothing here - read desired attitude values from
		// callback functions.
		euler2RotationMatrix(
				(double)attitude_reference_(0),
				(double)attitude_reference_(1),
				(double)attitude_reference_(2),
				R_d_);
	}

	// cout << "R_d:\n" << R_d_ << "\n";
	// cout << "R_mv:\n" << R_mv_ << "\n";

	// ATTITUDE TRACKING
	// Calculate control moment M
	e_R_skew = (R_d_.transpose() * R_mv_ - R_mv_.transpose() * R_d_) / 2;
	veeOperator(e_R_skew, e_R);
	e_omega = (omega_mv_ - R_mv_.transpose() * R_d_ * omega_d_);

	hatOperator(
			(double)omega_mv_(0, 0),
			(double)omega_mv_(1, 0),
			(double)omega_mv_(2, 0),
			omega_mv_skew);

	// Construct parameter matrix
	k_R_(0, 0) = params_.k_R_xy;
	k_R_(1, 1) = params_.k_R_xy;
	k_R_(2, 2) = params_.k_R_z;

	k_omega_(0, 0) = params_.k_omega_xy;
	k_omega_(1, 1) = params_.k_omega_xy;
	k_omega_(2, 2) = params_.k_omega_z;

	M_u = 	- k_R_ * e_R
			- k_omega_ * e_omega
			+ cross(omega_mv_, INERTIA * omega_mv_)
			- INERTIA *
			(
				omega_mv_skew * R_mv_.transpose() * R_d_ * omega_d_
				- R_mv_.transpose() * R_d_ * alpha_d_
			);

	M_u(0, 0) = saturation((double)M_u(0, 0), -MAXIMUM_MOMENT, MAXIMUM_MOMENT);
	M_u(1, 0) = saturation((double)M_u(1, 0), -MAXIMUM_MOMENT, MAXIMUM_MOMENT);
	M_u(2, 0) = saturation((double)M_u(2, 0), -MAXIMUM_MOMENT, MAXIMUM_MOMENT);
}

void UavGeometryControl::trajectoryTracking(
		const Matrix<double, 3, 1> pos_desired,
		const Matrix<double, 3, 1> pos_old,
		const double dt,
		Matrix<double, 3, 1> &b3_d,
		double &f_u)
{

	// Position errors
	Matrix<double, 3, 1> e_x, e_v;

	// TRAJECTORY TRACKING
	// Calculate total thrust and b3_d (desired thrust vector)
	if (current_control_mode_ == POSITION_CONTROL)
	{
		//cout << "Trajectory: POSITION" << "\n";
		e_x = (x_mv_ - pos_desired);
		e_v = (v_mv_ - v_d_);
	}
	else if (current_control_mode_ == ATTITUDE_CONTROL)
	{
		//cout << "Trajectory: ATTITUDE" << "\n";
		/**
		 * During Attitude control only take z - component of
		 * position and linear velocity.
		 */

		//v_d_ = (x_d_ - pos_old);
		e_x = (x_mv_(2, 0) - pos_desired(2, 0)) * E3;
		e_v = (v_mv_(2, 0) - v_d_(2, 0)) * E3;
	}

	k_x_(0, 0) = params_.k_position_xy;
	k_x_(1, 1) = params_.k_position_xy;
	k_x_(2, 2) = params_.k_position_z;

	k_v_(0, 0) = params_.k_velocity_xy;
	k_v_(1, 1) = params_.k_velocity_xy;
	k_v_(2, 2) = params_.k_velocity_z;

	// desired control force for the translational dynamics
	Matrix<double, 3, 1> A =
		- k_x_ * e_x
		- k_v_ * e_v
		+ UAV_MASS * G * E3
		+ UAV_MASS * a_d_;
	f_u = dot(A,  R_mv_ * E3 );
	b3_d = A / norm(A);
}

double saturation(
		double value,
		double lowLimit,
		double highLimit)
{
	if (value > highLimit) { return highLimit; }
	else if (value < lowLimit) { return lowLimit; }
	else { return value; }
}

Matrix<double, 3, 1> cross(
		Matrix<double, 3, 1> vec1,
		Matrix<double, 3, 1> vec2)
{
	Matrix<double, 3, 1> res;
	res(0, 0) = vec1(1, 0) * vec2(2, 0) - vec1(2, 0) * vec2(1, 0);
	res(1, 0) = vec1(2, 0) * vec2(0, 0) - vec1(0, 0) * vec2(2, 0);
	res(2, 0) = vec1(0, 0) * vec2(1, 0) - vec1(1, 0) * vec2(0, 0);
	return res;
}

double norm(Matrix<double, 3, 1> vec)
{
	return sqrt(dot(vec, vec));
}

double dot(Matrix<double, 3, 1> vec1, Matrix<double, 3, 1> vec2)
{
	return vec1(0, 0) * vec2(0, 0) +
			vec1(1, 0) * vec2(1, 0) +
			vec1(2, 0) * vec2(2, 0);
}

void vecSqrt(Matrix<double, 4, 1>& vec)
{
	vec(0, 0) = sqrt( (double)vec(0, 0) );
	vec(1, 0) = sqrt( (double)vec(1, 0) );
	vec(2, 0) = sqrt( (double)vec(2, 0) );
	vec(3, 0) = sqrt( (double)vec(3, 0) );
}

double signum(double val)
{
	if (val > 0) { return 1; }
	else if (val < 0) { return -1; }
	else { return 0; }
}

Matrix<double, 4, 1> vecSign(Matrix<double, 4, 1> vec)
{
	Matrix<double, 4, 1> res;
	res(0, 0) = signum( (double)vec(0, 0) );
	res(1, 0) = signum( (double)vec(1, 0) );
	res(2, 0) = signum( (double)vec(2, 0) );
	res(3, 0) = signum( (double)vec(3, 0) );
	return res;
}

void hatOperator(
		const double x,
		const double y,
		const double z,
		Matrix<double, 3, 3> &hatMatrix)
{
	hatMatrix.setZero();
	hatMatrix(0, 1) = -z;
	hatMatrix(0, 2) =  y;
	hatMatrix(1, 0) =  z;
	hatMatrix(1, 2) = -x;
	hatMatrix(2, 0) = -y;
	hatMatrix(2, 1) =  x;
}

void veeOperator(
		Matrix<double, 3, 3> hatMatrix,
		Matrix<double, 3, 1> &veeVector)
{
	veeVector.setZero();
	veeVector(0, 0) = hatMatrix(2, 1); 			// x component
	veeVector(1, 0) = hatMatrix(0, 2);			// y component
	veeVector(2, 0) = hatMatrix(1, 0);			// z component
}

void euler2RotationMatrix(
		const double roll,
		const double pitch,
		const double yaw,
		Matrix<double, 3, 3> &rotMatrix)
{
	rotMatrix.setZero();
	rotMatrix(0, 0) = cos(yaw) * cos(pitch);
	rotMatrix(0, 1) = cos(yaw) * sin(pitch) * sin(roll) - sin(yaw) * cos(roll);
	rotMatrix(0, 2) = cos(yaw) * sin(pitch) * cos(roll) + sin(yaw) * sin(roll);
	rotMatrix(1, 0) = sin(yaw) * cos(pitch);
	rotMatrix(1, 1) = sin(yaw) * sin(pitch) * sin(roll) + cos(yaw) * cos(roll);
	rotMatrix(1, 2) = sin(yaw) * sin(pitch) * cos(roll) - cos(yaw) * sin(roll);
	rotMatrix(2, 0) = - sin(pitch);
	rotMatrix(2, 1) = cos(pitch) * sin(roll);
	rotMatrix(2, 2) = cos(pitch) * cos(roll);
}
