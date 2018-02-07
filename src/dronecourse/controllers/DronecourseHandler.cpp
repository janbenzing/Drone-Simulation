/**
 * @file DronecourseHandler.cpp
 * Class to handle updates of dronecourse related controllers
 * Designed for the LIS-EPFL DroneCourse
 *
 * @author Basil Huber LIS <basil.huber@gmail.com>
 */

#include "DronecourseHandler.hpp"
#include <uORB/topics/dronecourse_velocity_setpoint.h>
#include <drivers/drv_hrt.h>


#include <iostream>

DronecourseHandler::DronecourseHandler() :
	_mode(DcMode::IDLE),
	_auto_mode(false),
	_pos_ctrl(_gimbal),
	_sonar_landing_ctrl(_gimbal),
	_follower(_gimbal),
	_trajectory_ctrl(_gimbal, _waypoint_navigator)
{
}

void DronecourseHandler::update()
{
	_gimbal.update();

	switch (_mode) {
	case DcMode::IDLE:
		break;

	case DcMode::POS_CTRL:
		_pos_ctrl.update();

		break;

	case DcMode::WAYPOINT_NAVIGATION:
		_trajectory_ctrl.update();

		// if we are in auto_mode and we reached the goal, continue with next mode
		if (_auto_mode &&_trajectory_ctrl.is_goal_reached()) {
			_mode = DcMode::SONAR_LANDING;
			PX4_INFO("Switching to SONAR_LANDING (automatically)");
		}

		break;

	case DcMode::SONAR_LANDING:
		_sonar_landing_ctrl.update();

		// if we are in auto_mode and we reached the goal, continue with next mode
		if (_sonar_landing_ctrl.is_goal_reached()) {
			// Successful landing on platform
			if (_auto_mode) {
				// Continue to next task
				_mode = DcMode::TARGET_FOLLOWING;
				PX4_INFO("Switching to TARGET_FOLLOWING (automatically)");
			} else {
				// Stop here
				_sonar_landing_ctrl.disarm();
				_mode = DcMode::IDLE;
				PX4_INFO("Switching to IDLE (automatically)");
			}
		}

		break;

	case DcMode::TARGET_FOLLOWING:
		_follower.update();

		// if we are in auto_mode and we reached the goal, continue with next mode
		if (_follower.is_goal_reached()) {
			// disarm
			_follower.disarm();
			_mode = DcMode::IDLE;
			PX4_INFO("Switching to IDLE (automatically)");
		}

		break;
	}
}

void DronecourseHandler::set_position_command(float x, float y, float z)
{
	matrix::Vector3f pos(x, y, z);
	_pos_ctrl.set_position_command(pos);
}
