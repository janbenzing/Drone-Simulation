include(cmake/configs/posix_sitl_lpe.cmake)

macro(config_dronecourse PATH_PREFIX)
list(APPEND config_module_list
	#
	# LIS modules
	#
	${PATH_PREFIX}/controllers
	${PATH_PREFIX}/target_tracking
	${PATH_PREFIX}/target_detection
	${PATH_PREFIX}/offboard_controller
	${PATH_PREFIX}/tests
	)
add_definitions(-D__DRONECOURSE)
endmacro()