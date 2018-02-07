/**
 * @file TrajectoryController.cpp
 * Class to handle predefined flight trajectory
 * Designed for the LIS-EPFL DroneCourse
 *
 * @author Basil Huber LIS <basil.huber@gmail.com>
 * @author Arthur Gay <arthur.gay@epfl.ch>
 */

#include "TrajectoryCtrl.hpp"

TrajectoryCtrl::TrajectoryCtrl(GimbalCtrl &gimbal, WaypointNavigator &navigator) :
	PositionCtrl(gimbal),
	_navigator(&navigator)
	// --------------------------------------------------------------------------
	// TODO: Initialize the waypoint index (initially invalid)
	// --------------------------------------------------------------------------
{
}


void TrajectoryCtrl::update()
{
	// --------------------------------------------------------------------------
	// TODO: Set the position command for the position controller
	// --------------------------------------------------------------------------

	// --------------------------------------------------------------------------
	// TODO: Upate the position controller
	// --------------------------------------------------------------------------
}

bool TrajectoryCtrl::is_goal_reached()
{
	// --------------------------------------------------------------------------
	// TODO: Implement the logic for having visited all waypoints
	// --------------------------------------------------------------------------
	return true;
}
