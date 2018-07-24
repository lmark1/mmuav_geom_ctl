/// \file morus_mmc_vpc_attitude_control_main.cpp
/// \brief Main starts and stops attitude control task.
///

#include "UavGeometryControl.h"

extern "C" __EXPORT int uav_geometry_control_main(int argc, char *argv[]);

int uav_geometry_control_main(int argc, char *argv[])
{
    // Show usage to user
    if (argc < 2) {
        warnx("usage: uav_geometry_control {start|stop|status}");
        return 1;
    }

    // Try to start the task
    if (!strcmp(argv[1], "start")) {

        if (uav_geometry_control::instance != nullptr) {
            warnx("already running");
            return 1;
        }

        uav_geometry_control::instance = new UavGeometryControl;

        // If there is not enough memory send info and exit
        if (uav_geometry_control::instance == nullptr) {
            warnx("alloc failed");
            return 1;
        }

        // If task couldn't be spawned
        if (OK != uav_geometry_control::instance->start()) {
            delete uav_geometry_control::instance;
            uav_geometry_control::instance = nullptr;
            warnx("start failed");
            return 1;
        }

        return 0;
    }

    // Stopping the task is esentially deleting it
    if (!strcmp(argv[1], "stop")) {
        if (uav_geometry_control::instance == nullptr) {
            warnx("not running");
            return 1;
        }

        delete uav_geometry_control::instance;
        uav_geometry_control::instance = nullptr;
        return 0;
    }

    if (!strcmp(argv[1], "status")) {
        if (uav_geometry_control::instance) {
            warnx("running");
            return 0;

        } else {
            warnx("not running");
            return 1;
        }
    }

    warnx("unrecognized command");
    return 1;
}
