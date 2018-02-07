/**
 * @file DronecourseHandler.hpp
 * Class to handle updates of dronecourse related controllers
 * Designed for the LIS-EPFL DroneCourse
 *
 * @author Basil Huber LIS <basil.huber@gmail.com>
 */

#pragma once
#include <px4_posix.h>
#include "PositionCtrl.hpp"
#include "SonarLandingCtrl.hpp"
#include "GimbalCtrl.hpp"
#include "TargetFollower.hpp"
#include "TrajectoryCtrl.hpp"
#include "waypoints.hpp"

class DronecourseHandler
{

public:

	enum class DcMode {
		IDLE,
		POS_CTRL,
		WAYPOINT_NAVIGATION,
		SONAR_LANDING,
		TARGET_FOLLOWING,
	};

	DronecourseHandler();

	~DronecourseHandler() {};

	void update();

	void set_mode(DcMode mode) {_mode = mode; _auto_mode = false; _mode_changed = true;};

	void set_mode_auto() {_auto_mode = true;};

	void set_position_command(float x, float y, float z);

	void set_yaw_command(float yaw);

	GimbalCtrl &gimbal() {return _gimbal;};

private:


	DcMode _mode;
	bool   _auto_mode;
	bool   _mode_changed;

	GimbalCtrl _gimbal;
	PositionCtrl _pos_ctrl;
	SonarLandingCtrl _sonar_landing_ctrl;
	TargetFollower _follower;
	WaypointNavigator _waypoint_navigator;
	TrajectoryCtrl _trajectory_ctrl;

};
