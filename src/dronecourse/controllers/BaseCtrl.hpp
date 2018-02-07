/**
 * @file BaseController.cpp
 * Base class for controllers
 * provides send_velocity_command(...) to send commands to mc_pos_control
 * that should be called from update
 * Designed for the LIS-EPFL DroneCourse
 *
 * @author Basil Huber LIS <basil.huber@gmail.com>
 */


#pragma once
#include <px4_posix.h>
#include <matrix/math.hpp>
#include "GimbalCtrl.hpp"

class BaseCtrl
{

public:
	BaseCtrl(GimbalCtrl &gimbal) :
		_vel_setpoint_pub(nullptr),
		_gimbal(gimbal)
		{};

	virtual ~BaseCtrl();

	virtual void update() = 0;

	virtual bool is_goal_reached() = 0;

	static void arm();

	static void disarm();

protected:
	void send_velocity_command(const matrix::Vector3f &vel_command, float yaw_command = NAN);

	orb_advert_t _vel_setpoint_pub;	// publication handle for dronecourse_velocity_setpoint

	GimbalCtrl &_gimbal;			// gimbal controller to set gimbal orientation

	static void arm_disarm(int armed);
};
