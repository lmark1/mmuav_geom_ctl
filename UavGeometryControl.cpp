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

	current_control_mode_(ATTITUDE_CONTROL)
{
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

	while(!geometry_control_task_should_exit_)
	{
		// Sleep and collect callback data
		geometry_control_loop_rate_.sleep();
		geometry_control_node_handle_.spinOnce();
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
	rates_measured_(0) = msg.data().rollspeed;
	rates_measured_(1) = msg.data().pitchspeed;
	rates_measured_(2) = msg.data().yawspeed;
}

void UavGeometryControl::parametersUpdateCallback(
		const px4::px4_parameter_update &msg)
{
  PX4_INFO("Parameters updated!");
  params_.update();
}

void UavGeometryControl::hatOperator(
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

void UavGeometryControl::veeOperator(
		Matrix<double, 3, 3> hatMatrix,
		Matrix<double, 3, 1> &veeVector)
{
	veeVector.setZero();
	veeVector(0, 0) = hatMatrix(2, 1); 			// x component
	veeVector(1, 0) = hatMatrix(0, 2);			// y component
	veeVector(2, 0) = hatMatrix(1, 0);			// z component
}

void UavGeometryControl::euler2RotationMatrix(
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
		Matrix<double, 3, 1> b13_normal = b3_desired.cross(b1_desired);

		// Compute b1_c = Proj[b1_d] onto the plane with normal b3_d
		b1_c = - b3_desired.cross(b13_normal) / b13_normal.norm();

		// Construct desired rotation matrix
		b2_c = b3_desired.cross(b1_c);
		b2_c = b2_c / b2_c.norm();
		R_c.setZero(3, 3);
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
				(double)euler_d_(0,0),
				(double)euler_d_(1,0),
				(double)euler_d_(2,0),
				R_d_);
	}
	else
	{
		ROS_ERROR("Invalid control mode given.");
		throw std::runtime_error("Invalid control mode given.");
	}

	cout << "R_d:\n" << R_d_ << "\n";
	cout << "R_mv:\n" << R_mv_ << "\n";

	// ATTITUDE TRACKING
	// Calculate control moment M
	e_R_skew = (R_d_.adjoint() * R_mv_ - R_mv_.adjoint() * R_d_) / 2;
	veeOperator(e_R_skew, e_R);
	e_omega = (omega_mv_ - R_mv_.adjoint() * R_d_ * omega_d_);
	if (e_omega(0, 0) != e_omega(0, 0))
	{
		throw std::runtime_error("STOP");
	}
	hatOperator(
			(double)omega_mv_(0, 0),
			(double)omega_mv_(1, 0),
			(double)omega_mv_(2, 0),
			omega_mv_skew);

	M_u = 	- k_R_ * e_R
			- k_omega_ * e_omega
			+ omega_mv_.cross(INERTIA * omega_mv_)
			- INERTIA *
			(
				omega_mv_skew * R_mv_.adjoint() * R_d_ * omega_d_
				- R_mv_.adjoint() * R_d_ * alpha_d_
			);

	M_u(0, 0) = saturation((double)M_u(0, 0), -MAXIMUM_MOMENT, MAXIMUM_MOMENT);
	M_u(1, 0) = saturation((double)M_u(1, 0), -MAXIMUM_MOMENT, MAXIMUM_MOMENT);
	M_u(2, 0) = saturation((double)M_u(2, 0), -MAXIMUM_MOMENT, MAXIMUM_MOMENT);
}

