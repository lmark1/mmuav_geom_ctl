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

	/**
	 * Calculate b3_d and f_u as position tracking control inputs.
	 *
	 * @param pos_desired - desired position reference
	 * 						(may change due to prefilter)
	 * @param pos_old - old position
	 * @param dt - time interval
	 * @param b3_d - thrust heading reference, assigned in method
	 * @param f_u - thrust magnitude value, assigned in method
	 */
	void trajectoryTracking(
			const Matrix<double, 3, 1> pos_desired,
			const Matrix<double, 3, 1> pos_old,
			const double dt,
			Matrix<double, 3, 1> &b3_d,
			double &f_u);

	/**
	 * Calculate control moments M_u used for attitude tracking.
	 *
	 * @param b1_d - desired heading
	 * @param b3_d - desired thrust vector
	 * @param dt - time interval
	 * @param R_c_old - reference for old calculated rotation matrix
	 * 					(Position tracking)
	 * @param omega_c_old - reference for old calculated angular velocity
	 * 						(Position tracking)
	 * @param M_u - control moments, assigned in method
	 */
	void attitudeTracking(
			const Matrix<double, 3, 1> b1_desired,
			const Matrix<double, 3, 1> b3_desired,
			const double dt,
			Matrix<double, 3, 3> &R_c_old,
			Matrix<double, 3, 3> &omega_c_old,
			Matrix<double, 3, 1> &M_u);

	/**
	 * Calculate and set desired angular velocity and
	 * desired angular acceleration based on R_c[k] and R_c[k-1].
	 *
	 * @param R_c - Calculated R
	 * @param R_c_old - Calculated old R
	 * @param R_mv - Measured R
	 * @param omega_c_old - Reference to old angular velocity value
	 * @param dt
	 */
	void calculateDesiredAngularVelAndAcc(
			const Matrix<double, 3, 3> R_c,
			const Matrix<double, 3, 3> R_c_old,
			const Matrix<double, 3, 3> R_mv,
			Matrix<double, 3, 3> &omega_c_old,
			double dt);

	/**
	 * Calculate rotor velocities from given vector containing
	 * total thrust and moments M_x, M_y, M_z.
	 *
	 * @param thrust_moment_vec
	 * @param transform_matrix - matrix transforms given vector to forces
	 * @rotor_velocities
	 */
	void calculateRotorVelocities(
			Matrix<double, 4, 1> thrust_moment_vec,
			Matrix<double, 4, 4> transform_matrix,
			Matrix<double, 4, 1>& rotor_velocities);


	/**
	 * Perform hat operator on given vector components.
	 *
	 * @param x - x vector component
	 * @param y - y vector component
	 * @param z - z vector component
	 * @param hatMatrixs - Matrix of the following form:
	 * 	[ 0  -z,  y]
	 * 	[ z,  0, -x]
	 * 	[-y,  x,  0]
	 */
	void hatOperator(
			const double x,
			const double y,
			const double z,
			Matrix<double, 3, 3> &hatMatrix);

	/**
	 * Perform a vee( V ) operator on a given hatMatrix.
	 * It is implied that the given hatMatrix is a skew matrix.
	 * It will decompose the skew matrix into a given veeVector reference.
	 *
	 * @param hatMatrx
	 * @param veeVector
	 */
	void veeOperator(
			const Matrix<double, 3, 3> hatMatrix,
			Matrix<double, 3, 1> &veeVector);
	/**
	 * Euler angles represented as a rotation matrix.
	 *
	 * @param roll
	 * @param pitch
	 * @param yaw
	 * @param rotMatrix - Rotation matrix will be stored here
	 */
	void euler2RotationMatrix(
			const double roll,
			const double pitch,
			const double yaw,
			Matrix<double, 3, 3> &rotMatrix);

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

	// Controller flags
	bool armed_flag_;
	bool roll_control_flag_previous_, roll_control_flag_;
	bool pitch_control_flag_previous_, pitch_control_flag_;
	bool yaw_control_flag_previous_, yaw_control_flag_;
	bool yaw_rate_control_flag_previous_, yaw_rate_control_flag_;

	// Parameters object
	UavGeometryControlParams params_;

	// Controller loop rate
	px4::Rate geometry_control_loop_rate_;

	// Geometry control task ID and running check
	int geometry_control_task_;
	bool geometry_control_task_should_exit_;


	/* Current control mode:
	 * 	- position control
	 * 	- attitude control
	 * 	- linear velocity control
	 */
	int current_control_mode_;

};

#endif /* UAV_GEOMETRY_CONTROL_H */
