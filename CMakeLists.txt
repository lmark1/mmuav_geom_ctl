px4_add_module(
	MODULE morus_modules__uav_geometry_control
	MAIN uav_geometry_control_main
	STACK_MAIN 1200 # IMPORTANT!!!, defines stack size for this part
	COMPILE_FLAGS
	SRCS
		uav_geometry_control_main.cpp
		UavGeometryControl.cpp
	DEPENDS
		platforms__common
	)
# vim: set noet ft=cmake fenc=utf-8 ff=unix :
