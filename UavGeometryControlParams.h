/*
 * UavGeometryControlParams.h
 *
 *  Created on: Jul 24, 2018
 *      Author: lmark
 */

#ifndef UAV_GEOMETRY_CONTROL_PARAMS_H
#define UAV_GEOMETRY_CONTROL_PARAMS_H

#include <systemlib/param/param.h>
//#include <uORB/uORB.h>
#include <px4_defines.h>
#include <px4.h>

class UavGeometryControlParams {

public:

	UavGeometryControlParams();
	~UavGeometryControlParams();

	/// Updates parameters through private handles.
	/// \brief Parameter update
	void update();

	// Controller sample rate
	int controller_sample_rate;

	// x / y position gain
	float k_position_xy;
	// z position gain
	float k_position_z;

	// x / y velocity gain
	float k_velocity_xy;
	// z velocity gain
	float k_velocity_z;

	// Roll / pitch attitude gain
	float k_R_xy;
	// Yaw attitude gain
	float k_R_z;

	// Roll / pitch rate gain
	float k_omega_xy;
	// Yaw rate gain
	float k_omega_z;

private:
	// Parameter handles are private members

	px4::ParameterInt controller_sample_rate_handle_;

	px4::ParameterFloat k_position_xy_handle_;
	px4::ParameterFloat k_position_z_handle_;

	px4::ParameterFloat k_velocity_xy_handle_;
	px4::ParameterFloat k_velocity_z_handle_;

	px4::ParameterFloat k_R_xy_handle_;
	px4::ParameterFloat k_R_z_handle_;

	px4::ParameterFloat k_omega_xy_handle_;
	px4::ParameterFloat k_omega_z_handle_;
};

#endif /* UAV_GEOMETRY_CONTROL_PARAMS_H */
