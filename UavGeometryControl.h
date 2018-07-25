/*
 * UavGeometryControl.h
 *
 *  Created on: Jul 23, 2018
 *      Author: lmark
 */

#ifndef UAV_GEOMETRY_CONTROL_H
#define UAV_GEOMETRY_CONTROL_H

// Pixhawk
#include <px4.h>

// Topics
#include <platforms/nuttx/px4_messages/px4_attitude_controller_reference.h>
#include <platforms/nuttx/px4_messages/px4_moving_mass_setpoint.h>
#include <platforms/nuttx/px4_messages/px4_moving_mass_setpoint_array.h>
#include <platforms/nuttx/px4_messages/px4_gas_motor_setpoint.h>
#include <platforms/nuttx/px4_messages/px4_gas_motor_setpoint_array.h>
#include <platforms/nuttx/px4_messages/px4_pid_status.h>
#include <platforms/nuttx/px4_messages/px4_attitude_controller_status.h>

// Standard
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>

// Drivers
#include <drivers/drv_hrt.h>

// Math library
#include <lib/mathlib/mathlib.h>
#include <matrix/math.hpp>
#include <matrix/filter.hpp>
#include <matrix/integration.hpp>

// Parameters
#include "UavGeometryControlParams.h"

class UavGeometryControl;

using namespace matrix;

namespace uav_geometry_control
{
  extern UavGeometryControl *instance;
}


class UavGeometryControl {
public:

	UavGeometryControl();
	~UavGeometryControl();

	/// Starts the position control task in background.
	int start();

protected:

	// Node handles for subscribers and publishers
	px4::NodeHandle geometry_control_node_handle_;
	px4::AppState geometry_control_app_state_;

private:

	// Geometry control task
	static void geometryControlTaskTrampoline(int argc, char *argv[]);
	void geometryControlTaskMain();

	void trajectoryTracking(
			const Matrix<double, 3, 1> pos_desired,
			const Matrix<double, 3, 1> pos_old,
			const double dt,
			Matrix<double, 3, 1> &b3_d,
			double &f_u);

	void attitudeTracking(
			const Matrix<double, 3, 1> b1_desired,
			const Matrix<double, 3, 1> b3_desired,
			const double dt,
			Matrix<double, 3, 3> &R_c_old,
			Matrix<double, 3, 3> &omega_c_old,
			Matrix<double, 3, 1> &M_u);

	void calculateDesiredAngularVelAndAcc(
			const Matrix<double, 3, 3> R_c,
			const Matrix<double, 3, 3> R_c_old,
			const Matrix<double, 3, 3> R_mv,
			Matrix<double, 3, 3> &omega_c_old,
			double dt);

	void calculateRotorVelocities(
			Matrix<double, 4, 1> thrust_moment_vec,
			Matrix<double, 4, 4> transform_matrix,
			Matrix<double, 4, 1>& rotor_velocities);

	// Publishers
	px4::Publisher<px4::px4_moving_mass_setpoint_array>
		*moving_mass_setpoint_array_pub_;
	px4::Publisher<px4::px4_gas_motor_setpoint_array>
		*gas_motor_setpoint_array_pub_;
	px4::Publisher<px4::px4_attitude_controller_status>
		*attitude_controller_status_pub_;

	// Topics callbacks
	void vehicleAttitudeCallback(
			const px4::px4_vehicle_attitude &msg);
	void parametersUpdateCallback(
			const px4::px4_parameter_update &msg);
	void attitudeReferenceCallback(
			const px4::px4_attitude_controller_reference &msg);

	// Angle setpoints and measured value
	float rotor_speed_reference_, yaw_rate_feedforward_;
	math::Vector<3> attitude_reference_;
	math::Vector<3> attitude_measured_;
	math::Vector<3> rates_measured_;

	matrix::Matrix<double, 3, 3> R_d_, R_mv_;
	matrix::Matrix<double, 3, 1> omega_mv_, omega_d_, alpha_d_;
	matrix::Matrix<double, 3, 1> x_mv_, x_d_, v_mv_, v_d_, a_d_, b1_d_;

	// Controller flags
	bool armed_flag_;
	bool roll_control_flag_previous_, roll_control_flag_;
	bool pitch_control_flag_previous_, pitch_control_flag_;
	bool yaw_control_flag_previous_, yaw_control_flag_;
	bool yaw_rate_control_flag_previous_, yaw_rate_control_flag_;
	bool enable_mass_control_;

	// Parameters object
	UavGeometryControlParams params_;

	// Controller loop rate
	px4::Rate geometry_control_loop_rate_;

	// Geometry control task ID and running check
	int geometry_control_task_;
	bool geometry_control_task_should_exit_;

	/**
	 * CONTROLLER PARAMETERS:
	 *	- k_x: position tracking error gain (eX)
	 *	- k_v: velocity tracking error gain (eV)
	 *	- k_R: orientation matrix error gain (eR)
	 *	- k_omega: angular velocity error gain (eOmega)
	 */
	Matrix<double, 3, 3> k_x_, k_v_, k_R_, k_omega_;

	/* Current control mode:
	 * 	- position control
	 * 	- attitude control
	 * 	- linear velocity control
	 */
	int current_control_mode_;

};

double saturation(
		double value,
		double lowLimit,
		double highLimit);

double norm(Matrix<double, 3, 1> vec);

Matrix<double, 3, 1> cross(
		Matrix<double, 3, 1> vec1,
		Matrix<double, 3, 1> vec2);

double dot(
		Matrix<double, 3, 1> vec1,
		Matrix<double, 3, 1> vec2);

void vecSqrt(Matrix<double, 4, 1>& vec);

Matrix<double, 4, 1> vecSign(Matrix<double, 4, 1> vec);

double signum(double val);

void hatOperator(
			const double x,
			const double y,
			const double z,
			Matrix<double, 3, 3> &hatMatrix);

void veeOperator(
		const Matrix<double, 3, 3> hatMatrix,
		Matrix<double, 3, 1> &veeVector);

void euler2RotationMatrix(
		const double roll,
		const double pitch,
		const double yaw,
		Matrix<double, 3, 3> &rotMatrix);

#endif /* UAV_GEOMETRY_CONTROL_H */
