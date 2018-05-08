/**
 * @file TrajectoryController.hpp
 * Class to handle predefined flight trajectory
 * Designed for the LIS-EPFL DroneCourse
 *
 * @author Basil Huber LIS <basil.huber@gmail.com>
 * @author Arthur Gay <arthur.gay@epfl.ch>
 */

#pragma once

#include <px4_posix.h>
#include "PositionCtrl.hpp"
#include <vector>
#include "waypoints.hpp"

class TrajectoryCtrl : public PositionCtrl
{
public:
	TrajectoryCtrl(GimbalCtrl &gimbal, WaypointNavigator &navigator);

	void update();

	virtual bool is_goal_reached();

private:

	WaypointNavigator *_navigator;
	orb_advert_t _dronecourse_waypoint_pub; 
	bool waypoint_published;
	void publish_waypoint_list(void);

	// --------------------------------------------------------------------------
	// TODO: Create member variables for the waypoint and its current index 
	// --------------------------------------------------------------------------
	int _waypoint_index;
	matrix::Vector3f _waypoint;
};
