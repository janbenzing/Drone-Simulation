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

#include <iostream>

TargetFollower::TargetFollower(GimbalCtrl &gimbal) :
	PositionCtrl(gimbal)
{
}


void TargetFollower::update()
{
}


bool TargetFollower::is_goal_reached() const
{
	return true;
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
