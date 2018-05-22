/**
 * @file TargetFollower.cpp
 * Class to follow moving target
 * Designed for the LIS-EPFL DroneCourse
 *
 * @author Basil Huber LIS <basil.huber@gmail.com>
 */

#include "TargetFollower.hpp"
#include <drivers/drv_hrt.h>
#include <px4_time.h>
#include <uORB/topics/target_position_ned_filtered.h>

#include <iostream>

TargetFollower::TargetFollower(GimbalCtrl &gimbal) :
	PositionCtrl(gimbal),
	_gimbal(gimbal),
	Etat(0)
{
	target_position_net_filtered_sub = orb_subscribe(ORB_ID(target_position_ned_filtered));
}


void TargetFollower::update()
{	
	TargetFollower::update_subscriptions();
	TargetFollower::update_parameters();
	switch(Etat)
	{
		case 0:
		{
		matrix::Vector3f obs_pos;
		obs_pos(0) = 0;
		obs_pos(1) = 50;
		obs_pos(2) = -10;

		PositionCtrl::set_position_command(obs_pos);
		PositionCtrl::update();

		if (PositionCtrl::is_goal_reached())
		{
			Etat = 1;
		}
		break;
		}

		case 1:
		{
			yaw = yaw + 0.015;
			pitch = -0.3;
			_gimbal.set_command(pitch,yaw);
			PositionCtrl::update();

			target_position_ned_filtered_s target_position;
	 		orb_copy(ORB_ID(target_position_ned_filtered), target_position_net_filtered_sub, &target_position);
	 		_target_filtered_x = target_position.x;
	 		_target_filtered_y = target_position.y;
	 		_target_filtered_z = target_position.z;
	 		_target_filtered_vx = target_position.vx;
	 		_target_filtered_vy = target_position.vy;
	 		_target_filtered_vz = target_position.vz;
	 		_target_filtered_var_x = target_position.var_x;
	 		_target_filtered_var_y = target_position.var_y;
	 		_target_filtered_var_z = target_position.var_z;

	 		PX4_INFO("VAR_X EST = %f", (double)_target_filtered_var_x);


			break;
		}
		case 2:
		{

			break;
		}
		case 3:
		{

			break;
		}

	}
}


bool TargetFollower::is_goal_reached() const
{
	return false;
}

void TargetFollower::update_subscriptions()
{
	// ------------------------------------------------------
	// TODO update subscriptions
	// ------------------------------------------------------

}

void TargetFollower::update_parameters()
{
	// ------------------------------------------------------
	// TODO update parameters
	// ------------------------------------------------------

}
