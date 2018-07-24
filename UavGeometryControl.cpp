/*
 * UavGeometryControl.cpp
 *
 *  Created on: Jul 23, 2018
 *      Author: lmark
 */

#include "UavGeometryControl.h"

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
	yaw_rate_control_flag_(attitude_controller_reference_s::OFF)
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


