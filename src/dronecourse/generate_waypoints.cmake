# Generate the waypoints.hpp file when
#   the waypoints.hpp file is needed but does not exist yet
#   or the world file has changed.
set(GENERATED_WAYPOINTS_HPP_INCLUDE_DIR ${CMAKE_BINARY_DIR}/waypoints)
set(GENERATED_WAYPOINTS_HPP ${GENERATED_WAYPOINTS_HPP_INCLUDE_DIR}/waypoints.hpp)
set(WORLD_FILE ${CMAKE_SOURCE_DIR}/Tools/sitl_gazebo/worlds/dronecourse.world)
set(GENERATOR ${CMAKE_SOURCE_DIR}/Tools/show-waypoints.sh)
add_custom_command(
	OUTPUT ${GENERATED_WAYPOINTS_HPP}
	COMMAND ${GENERATOR} -c -w ${WORLD_FILE} > ${GENERATED_WAYPOINTS_HPP}
	#DEPENDS ${WORLD_FILE} ${GENERATOR}
	COMMENT "Generating ${GENERATED_WAYPOINTS_HPP}"
)
