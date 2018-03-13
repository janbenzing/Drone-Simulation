/**
 * @file DronecourseHandler.cpp
 * Class to handle updates of dronecourse related controllers
 * Designed for the LIS-EPFL DroneCourse
 *
 * @author Basil Huber LIS <basil.huber@gmail.com>
 */

#include "DronecourseHandler.hpp"
#include <uORB/topics/dronecourse_velocity_setpoint.h>
#include <uORB/topics/dronecourse_status.h>
#include <uORB/topics/dronecourse_timeouts.h>
#include <uORB/topics/vehicle_land_detected.h>

#include <iostream>

#include <ctime>
#include <cstdlib>

DronecourseHandler::DronecourseHandler() :
	_land_detected_sub(orb_subscribe(ORB_ID(vehicle_land_detected))),
	_mode(DcMode::IDLE),
	_auto_mode(false),
	_timeout(10000000*60), // 10min
	_time_at_start(hrt_absolute_time()),
	_time_task1_start(0),
	_time_task2_start(0),
	_time_task3_start(0),
	_timeout_task1(false),
	_timeout_task2(false),
	_timeout_task3(false),
	_takeoff_task3(false),
	_task1_wait(10000000), // 10sec
	_task2_wait(0),
	_task3_wait(0), // 0sec (not needed anymore)
	_time_task1_completed(0),
	_time_task2_completed(0),
	_time_task3_completed(0),
	_task1_completed(false),
	_task2_completed(false),
	_task3_completed(false),
	_task1_completed_after_wait(false),
	_task2_completed_after_wait(false),
	_task3_completed_after_wait(false),
	_time_platform_takeoff(10000000), // 10sec
	_pos_ctrl(_gimbal),
	_sonar_landing_ctrl(_gimbal),
	_follower(_gimbal),
	_trajectory_ctrl(_gimbal, _waypoint_navigator),
	_dronecourse_status_pub(nullptr),
	_dronecourse_timeouts_pub(nullptr)
{
}

void DronecourseHandler::update()
{
	bool _timeout_flag = false;

	// Update gimbal controller
	_gimbal.update();

	// State machine
	switch (_mode) {
		case DcMode::IDLE:
			break;

		case DcMode::END:
			break;

		case DcMode::POS_CTRL:
			_pos_ctrl.update();
			break;

		case DcMode::WAYPOINT_NAVIGATION:
			_trajectory_ctrl.update();

			_timeout_flag = false;

			// if in automode, check timeout
			if (_auto_mode) {
				if (hrt_absolute_time() - _time_task1_start > _timeout){
					_timeout_flag = true;
				}
			}
			
			// if we are in auto_mode and we reached the goal, wait for # seconds in order for the task to be completed properly
			if (_auto_mode && _trajectory_ctrl.is_goal_reached() && !_task1_completed) {
				_time_task1_completed = hrt_absolute_time();
				_task1_completed = true;
				
				PX4_INFO("waiting 10 seconds to reach waypoint center"); //in seconds
			}

			if (_task1_completed) {
				if (hrt_absolute_time() >= (_time_task1_completed + _task1_wait)){
					_task1_completed_after_wait = true;
				}
			}

			// if timeout or waited for # seconds after completion, we switch to next task
			if (_task1_completed_after_wait || _timeout_flag) {
				_mode = DcMode::SONAR_LANDING;
				_time_task2_start = hrt_absolute_time();
				if (_timeout_flag){
					PX4_INFO("Timeout for WAYPOINT_NAVIGATION reached");
						_timeout_task1 = true;
				}
				PX4_INFO("Switching to SONAR_LANDING (automatically)");
			}

			break;

		case DcMode::SONAR_LANDING:
			_sonar_landing_ctrl.update();

			_timeout_flag = false;
			// if in automode, check timeout
			if (_auto_mode) {
				if (hrt_absolute_time() - _time_task2_start > _timeout){
					_timeout_flag = true;
				}
			}

			// if we are in auto_mode and we reached the goal, continue with next mode
			if (_sonar_landing_ctrl.is_goal_reached() || _timeout_flag) {
				// Successful landing on platform
				if (_auto_mode) {
					// Continue to next task
					_mode = DcMode::TARGET_FOLLOWING;
					_time_task3_start = hrt_absolute_time();
					if (_timeout_flag){
						PX4_INFO("Timeout for SONAR_LANDING reached");
						_timeout_task2 = true;
					}
					PX4_INFO("Switching to TARGET_FOLLOWING (automatically)");
				} 
				else {
					// Stop here
					_sonar_landing_ctrl.disarm();
					_mode = DcMode::IDLE;
					if (_timeout_flag){
						PX4_INFO("Timeout for TARGET_FOLLOWING reached");
					}
					PX4_INFO("Switching to IDLE (automatically)");
				}
			}

			break;

		case DcMode::TARGET_FOLLOWING:
			_follower.update();

			_timeout_flag = false;

			// if in automode, check timeout
			if (_auto_mode) {
				if (hrt_absolute_time() - _time_task3_start > _timeout){
					_timeout_flag = true;
				}
			}

			// if in automode and no timeout for task 2, wait from new takeoff (it will be already landed)
			if (_auto_mode && !_timeout_task2 && !_takeoff_task3) {
				vehicle_land_detected_s land_detected;
				orb_copy(ORB_ID(vehicle_land_detected), _land_detected_sub, &land_detected);

				if (land_detected.landed) {
					PX4_INFO("waiting for takeoff");
					break;
				}
				else{
					_takeoff_task3 = true;
				}
			}

			// if timeout or target completed, we wait for some seconds to log
			if ((_follower.is_goal_reached() || _timeout_flag) && !_time_task3_completed) {
				// disarm
				_time_task3_completed = hrt_absolute_time();
				_task3_completed = true;
				if (_timeout_flag){
					PX4_INFO("Timeout for TARGET_FOLLOWING reached");
					_timeout_task3 = true;
				}
				PX4_INFO("waiting for some seconds before disarm to log your position on truck");

				// publishing timout flags

				dronecourse_timeouts_s dronecourse_timeouts;
				dronecourse_timeouts.timestamp = hrt_absolute_time();
				dronecourse_timeouts.timeout_task1 = _timeout_task1;
				dronecourse_timeouts.timeout_task2 = _timeout_task2;
				dronecourse_timeouts.timeout_task3 = _timeout_task3;
				if (_dronecourse_timeouts_pub == nullptr) {
					_dronecourse_timeouts_pub = orb_advertise(ORB_ID(dronecourse_timeouts), &dronecourse_timeouts);
				} else {
					orb_publish(ORB_ID(dronecourse_timeouts), _dronecourse_timeouts_pub, &dronecourse_timeouts);
				}
			}

			// if timer expired, we switch to next task
			if (_task3_completed && (hrt_absolute_time() >= (_time_task3_completed + _task3_wait))){
				_mode = DcMode::END;
				_follower.disarm();
				PX4_INFO("Switching to END (automatically)");
			}

			break;
	}

	// Publish current status for logging
	publish_dronecourse_status();
}


void DronecourseHandler::set_position_command(float x, float y, float z)
{
	matrix::Vector3f pos(x, y, z);
	_pos_ctrl.set_position_command(pos);
}


void DronecourseHandler::publish_dronecourse_status(void)
{
	dronecourse_status_s dronecourse_status;
	dronecourse_status.timestamp = hrt_absolute_time();
	dronecourse_status.mode = (int)_mode;
	dronecourse_status.autocontinue = _auto_mode;

	// publish dronecourse status
	if (_dronecourse_status_pub == nullptr) {
		_dronecourse_status_pub = orb_advertise(ORB_ID(dronecourse_status), &dronecourse_status);
	} else {
		orb_publish(ORB_ID(dronecourse_status), _dronecourse_status_pub, &dronecourse_status);
	}
}
