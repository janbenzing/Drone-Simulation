/**
 * @file SonarLandingCtrl.cpp
 * Class to control the drone for landing on a platform using a sonar.
 * Designed for the LIS-EPFL DroneCourse
 *
 * @author Arthur Gay <arthur.gay@epfl.ch>
 */

#include "SonarLandingCtrl.hpp"
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/Subscription.hpp>

#include <drivers/drv_hrt.h>

#include <iostream>

SonarLandingCtrl::SonarLandingCtrl(GimbalCtrl &gimbal) :
	PositionCtrl(gimbal)
	// ------------------------------------------------------
	// TODO subscribe to distance_sensor
	// HINT use the initializer list by completing the next statement
	// ------------------------------------------------------
	// _distance_sensor_sub(...)
{}

void SonarLandingCtrl::update()
{
	update_parameters();
	update_subscriptions();

	// ------------------------------------------------------
	// TODO print distance sensor
	// ------------------------------------------------------


	// ------------------------------------------------------
	// TODO implement platform detection algorithm
	// ------------------------------------------------------
	_platform_detected = false;


	// If the platform has not been found yet,
	// update the search algorithm
	if (!_platform_found) {
		_platform_found = update_search();

	} else { // else start the landing
		update_landing();
	}
}

void SonarLandingCtrl::update_subscriptions()
{
	// ------------------------------------------------------
	// TODO check distance sensor
	// ------------------------------------------------------


	// ------------------------------------------------------
	// TODO check land detection sensor
	// ------------------------------------------------------
}

void SonarLandingCtrl::update_parameters()
{
}

bool SonarLandingCtrl::is_goal_reached()
{
	// ------------------------------------------------------
	// TODO return true when the drone has landed
	// ------------------------------------------------------
	return true;
}


bool SonarLandingCtrl::update_search()
{
	// ------------------------------------------------------
	// TODO implement algorithm to search for the platform
	// ------------------------------------------------------
	return false; // return true when platform has been found
}

bool SonarLandingCtrl::update_landing()
{
	// ------------------------------------------------------
	// TODO implement controller for landing
	// ------------------------------------------------------
	return false; // return true when landed
}
