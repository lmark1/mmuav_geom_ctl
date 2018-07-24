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

};

#endif /* UAV_GEOMETRY_CONTROL_H */
