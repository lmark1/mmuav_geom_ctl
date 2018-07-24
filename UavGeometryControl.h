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

// Standard
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>


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
	px4::NodeHandle attitude_control_node_handle_;
	px4::AppState attitude_control_app_state_;

};

#endif /* UAV_GEOMETRY_CONTROL_H */
