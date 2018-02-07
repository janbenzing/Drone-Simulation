/**
 * @file GimbalCtrl.cpp
 * Class to send to control camera gimbal
 * Designed for the LIS-EPFL DroneCourse
 *
 * @author Basil Huber LIS <basil.huber@gmail.com>
 */

#include "GimbalCtrl.hpp"
#include <uORB/topics/gimbal_command.h>
#include <uORB/topics/target_position_ned_filtered.h>

#include <matrix/math.hpp>
#include <matrix/Euler.hpp>

#include <iostream>

GimbalCtrl::GimbalCtrl()
{
	// --------------------------------------------------
	// TODO subscribe to uORB messages:
	//      target_position_ned_filtered,
	//      vehicle_attitude and vehicle_local_position
	// --------------------------------------------------

	// set publishing handle for gimbal_command to nullptr
	_gimbal_command_pub = nullptr;

	// Set inital angles downward facing
	set_command(-M_PI / 2, 0.0f);
}

void GimbalCtrl::set_command(float pitch, float yaw)
{
	_mode = MODE::MANUAL;
	gimbal_command_s gimbal_command_msg;
	gimbal_command_msg.yaw = yaw;
	gimbal_command_msg.pitch = pitch;
	int instance;
	orb_publish_auto(ORB_ID(gimbal_command), &_gimbal_command_pub, &gimbal_command_msg, &instance, ORB_PRIO_HIGH);
}

void GimbalCtrl::update()
{
	if (_mode == MODE::AUTOMATIC) {
		// ------------------------------------------------
		// TODO create local variables for target position,
		//      drone position and drone attitude and
		//      set values from uORB
		// ------------------------------------------------

		// ------------------------------------------------
		// TODO create variable for target direction
		//      i.e., vector from drone to target
		// ------------------------------------------------

		// ------------------------------------------------
		// TODO create variables for drone's Z and Y axis
		//      in local frame
		// ------------------------------------------------

		// ---------------------------------------------------
		// TODO find normal vector of auxiliary plane
		//      containing target direction and
		//      drone's Z axis in local frame
		// ---------------------------------------------------

		// ---------------------------------------------------------------------
		// TODO find desired yaw angle as angle between
		//      the plane's normal vector and the drone's Y axis in local frame
		// ---------------------------------------------------------------------

		// ----------------------------------------------------------
		// TODO find desired pitch angle as angle between
		//      drone's Z axis in local frame and target direction
		// ----------------------------------------------------------

		// --------------------------------------------------------
		// TODO publish gimbal_command over uORB
		// --------------------------------------------------------
	}
}
