/*
 * UavGeometryControlParams.cpp
 *
 *  Created on: Jul 24, 2018
 *      Author: lmark
 */

#include "UavGeometryControlParams.h"

UavGeometryControlParams::UavGeometryControlParams() :

	controller_sample_rate_handle_("MV_SAMPLE_RATE", 0),

	k_position_xy_handle_("MV_POSXY_KP", 0),
	k_position_z_handle_("MV_POSZ_KP", 0),

	k_velocity_xy_handle_("MV_VELXY_KP", 0),
	k_velocity_z_handle_("MV_VELZ_KP", 0),

	k_R_xy_handle_("MV_RXY_KP", 0),
	k_R_z_handle_("MV_RZ_KP", 0),

	k_omega_xy_handle_("MV_OMEGAXY_KP", 0),
	k_omega_z_handle_("MV_OMEGAZ_KP", 0)
{
	update();

}

UavGeometryControlParams::~UavGeometryControlParams()
{

}

void UavGeometryControlParams::update()
{
	controller_sample_rate = controller_sample_rate_handle_.update();

	k_position_xy = k_position_xy_handle_.update();
	k_position_z = k_position_z_handle_.update();

	k_velocity_xy = k_velocity_xy_handle_.update();
	k_velocity_z = k_velocity_z_handle_.update();

	k_R_xy = k_R_xy_handle_.update();
	k_R_z = k_R_z_handle_.update();

	k_omega_xy = k_omega_xy_handle_.update();
	k_omega_z = k_omega_z_handle_.update();
}
