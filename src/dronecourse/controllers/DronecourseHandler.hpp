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

#include <drivers/drv_hrt.h>

class DronecourseHandler
{

public:

	enum class DcMode {
		IDLE,
		WAYPOINT_NAVIGATION,
		SONAR_LANDING,
		TARGET_FOLLOWING,
		POS_CTRL,
		END,
	};

	DronecourseHandler();

	~DronecourseHandler() {};

	void update();

	void set_mode(DcMode mode) {_mode = mode; _auto_mode = false; _mode_changed = true;};
	const DcMode& get_mode() const {return _mode;};

	void set_auto_mode(bool auto_mode) {_auto_mode = auto_mode;};

	void start_timer_task1() {_time_task1_start = hrt_absolute_time();};

	bool get_auto_mode() const {return _auto_mode;};
	
	void set_position_command(float x, float y, float z);

	void set_yaw_command(float yaw);

	GimbalCtrl &gimbal() {return _gimbal;};

private:


	int _land_detected_sub;
	DcMode   _mode;
	bool     _auto_mode;
	bool     _mode_changed;
	uint64_t _timeout;
	uint64_t _time_at_start;

	uint64_t _time_task1_start;
	uint64_t _time_task2_start;
	uint64_t _time_task3_start;
	bool _timeout_task1;
	bool _timeout_task2;
	bool _timeout_task3;
	bool _takeoff_task3;

	uint64_t _task1_wait;
	uint64_t _task2_wait;
	uint64_t _task3_wait;

	uint64_t _time_task1_completed;
	uint64_t _time_task2_completed;
	uint64_t _time_task3_completed;

	bool _task1_completed;
	bool _task2_completed;
	bool _task3_completed;

	bool _task1_completed_after_wait;
	bool _task2_completed_after_wait;
	bool _task3_completed_after_wait;

	uint64_t _time_platform_takeoff;

	GimbalCtrl _gimbal;
	PositionCtrl _pos_ctrl;
	SonarLandingCtrl _sonar_landing_ctrl;
	TargetFollower _follower;
	WaypointNavigator _waypoint_navigator;
	TrajectoryCtrl _trajectory_ctrl;

	void publish_dronecourse_status(void);

protected:

	orb_advert_t _dronecourse_status_pub; // publication handle for mission_phase
	orb_advert_t _dronecourse_timeouts_pub; // publication handle for mission_phase
};
