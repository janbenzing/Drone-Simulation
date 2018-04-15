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
	PositionCtrl(gimbal),
	search_counter(0),
	// ------------------------------------------------------
	// TODO subscribe to distance_sensor
	// HINT use the initializer list by completing the next statement
	// ------------------------------------------------------
	_local_pos_sub(orb_subscribe(ORB_ID(vehicle_local_position))),
	_distance_sensor_sub(orb_subscribe(ORB_ID(distance_sensor))),
	_ground_contact_sub(orb_subscribe(ORB_ID(vehicle_land_detected)))
	// _distance_sensor_sub(...)
{}

void SonarLandingCtrl::update()
{
	update_parameters();
	update_subscriptions();

	// ------------------------------------------------------
	// TODO print distance sensor
	// ------------------------------------------------------
	//PX4_INFO("Distance sensor is = %f", (double)_current_distance);
	//PX4_INFO("Current Z position is = %f", (double)_current_pos(2));

	// ------------------------------------------------------
	// TODO implement platform detection algorithm
	// ------------------------------------------------------
	_platform_detected = false;
	is_goal_reached();


	// If the platform has not been found yet,
	// update the search algorithm
	if (!_platform_found) {
		PX4_INFO("Update search");
		_platform_found = update_search();

	} 
	else { // else start the landing
		//PX4_INFO("START LANDING");		
		update_landing();
	}
}

void SonarLandingCtrl::update_subscriptions()
{
	
	// ------------------------------------------------------
	// TODO check distance sensor
	// ------------------------------------------------------
	bool updated_dist;

	orb_check(_distance_sensor_sub, &updated_dist);
	
	if (updated_dist) {
		distance_sensor_s current_sensor;
		orb_copy(ORB_ID(distance_sensor), _distance_sensor_sub, &current_sensor);
		_current_distance = current_sensor.current_distance;
	}

	// --------------------------------------------------------------------------
	// TODO: Check if drone's local position topic has been updated
	// --------------------------------------------------------------------------
	bool updated_pos;

	orb_check(_local_pos_sub, &updated_pos);

	if (updated_pos) {

		// ------------------------------------------------------------------------
		// TODO: Update current position member variable with new position data
		// ------------------------------------------------------------------------
		vehicle_local_position_s local_pos;
		orb_copy(ORB_ID(vehicle_local_position), _local_pos_sub, &local_pos);
		_current_pos(0) = local_pos.x;
		_current_pos(1) = local_pos.y;
		_current_pos(2) = local_pos.z;
	}

	// ------------------------------------------------------
	// TODO check land detection sensor
	// ------------------------------------------------------

	/////////////////////////////////////////////////////////PRINT NE MARCHE PAS (ground_contact ne se met jamais a 1)
	bool updated;

	orb_check(_ground_contact_sub, &updated);

	if (updated) {

		vehicle_land_detected_s landing;

		orb_copy(ORB_ID(vehicle_land_detected), _ground_contact_sub, &landing);
		PX4_INFO("landing.ground_contact = %d", landing.ground_contact);
		if (landing.ground_contact)
		{
			PX4_INFO("WE HAVE LANDED");
			_landed = true;
		}
	}
	/////////////////////////////////////////////////////////
}

void SonarLandingCtrl::update_parameters()
{
}

bool SonarLandingCtrl::is_goal_reached()
{
	// ------------------------------------------------------
	// TODO return true when the drone has landed
	// ------------------------------------------------------
	/*if (((double)(_current_pos(2) + _current_distance) < (-0.1)) && ((double)(_current_pos(2) + _current_distance) > (-4.2)) && (((double)_current_pos(2) < (-0.8))) && ((double)_current_pos(2) > (-6.5)))
	{
		_platform_detected = true;
		//PX4_INFO("Plateforme detected"); ATTENTION PARFOIS IL DETECTE ALORS QU'IL NE DEVRAIT PAS
	}*/
	return false;
}


bool SonarLandingCtrl::update_search()
{
	// ------------------------------------------------------
	// TODO implement algorithm to search for the platform
	// ------------------------------------------------------
	//PositionCtrl::set_position_command(&pos);

	if (((double)(_current_pos(2) + _current_distance) < (-0.1)) && ((double)(_current_pos(2) + _current_distance) > (-4.2)) && (((double)_current_pos(2) < (-0.8))) && ((double)_current_pos(2) > (-6.5)))
	{
		_platform_detected = true;
		PX4_INFO("Plateforme detected"); //ATTENTION PARFOIS IL DETECTE ALORS QU'IL NE DEVRAIT PAS
	}



	PositionCtrl::update();

	PX4_INFO("update search switch");

	switch (search_counter)
	{
		case 0:
			pos(0) = 0.0;
			pos(1) = 97.0;
			pos(2) = -3.0;

			PositionCtrl::set_position_command(pos);
			//PositionCtrl::update();

			PX4_INFO("search_counter = %d", search_counter);

			if (PositionCtrl::is_goal_reached())
			{
				search_counter = search_counter + 1;
			}
		return false;


		case 1:
			pos(0) = 0.0f;
			pos(1) = 104.0f;
			pos(2) = -3.0f;

			PositionCtrl::set_position_command(pos);
			//PositionCtrl::update();

			PX4_INFO("search_counter = %d", search_counter);

			if(_platform_detected)
			{
				//orb_copy(ORB_ID(vehicle_local_position), _local_pos_sub, &local_pos);
						
				_current_pos_save_1 = _current_pos(1);
						
				search_counter = search_counter + 1;
			}
				
		return false;

		case 2:

			pos(0) = 0.0f;
			pos(1) = 104.0f;
			pos(2) = -3.0f;			

			PositionCtrl::set_position_command(pos);
			//PositionCtrl::update();

			PX4_INFO("search_counter = %d", search_counter);

			//vehicle_local_position_s local_pos_save_2;

				if (not _platform_detected)
				{
					_current_pos_save_2 = _current_pos(1);

					search_counter = search_counter + 1;
				}
		return false;


		case 3:

			pos(0) = -4.0f;
			pos(1) = 100.0f;
			pos(2) = -3.0f;	

			PositionCtrl::set_position_command(pos);
			//PositionCtrl::update();

			PX4_INFO("search_counter = %d", search_counter);

				if (PositionCtrl::is_goal_reached())
				{
					search_counter = search_counter + 1;
				}
		return false;

		case 4:

			pos(0) = 4.0f;
			pos(1) = 100.0f;
			pos(2) = -3.0f;	

			PositionCtrl::set_position_command(pos);
			//PositionCtrl::update();

			PX4_INFO("search_counter = %d", search_counter);

			//vehicle_local_position_s local_pos_save_3;

				if(_platform_detected)
					{
						//orb_copy(ORB_ID(vehicle_local_position), _local_pos_sub, &local_pos);
						
						_current_pos_save_3 = _current_pos(0);
						
						search_counter = search_counter + 1;
					}
		return false;
					
		case 5:

			pos(0) = 4.0f;
			pos(1) = 100.0f;
			pos(2) = -3.0f;

			PositionCtrl::set_position_command(pos);
			//PositionCtrl::update();

			PX4_INFO("search_counter = %d", search_counter);

			if(not _platform_detected)
			{
				_current_pos_save_4 = _current_pos(0);

				search_counter = search_counter + 1;
			}
		return false;

		case 6:

			pos_landing_x = (_current_pos_save_4 + _current_pos_save_3)/2;

			pos_landing_y = (_current_pos_save_2 + _current_pos_save_1)/2;

			pos(0) = pos_landing_x;
			pos(1) = pos_landing_y;
			pos(2) = -3.0f;

			PositionCtrl::set_position_command(pos);

			PX4_INFO("search_counter = %d", search_counter);

			if (PositionCtrl::is_goal_reached())
				{
					PX4_INFO("WE CAN LAND");
					return true;
				}
			else
				{
					return false;
				}



	}

	return false; // return true when platform has been found
}

bool SonarLandingCtrl::update_landing()
{

	//bool updated;
	// ------------------------------------------------------
	// TODO implement controller for landing
	// ------------------------------------------------------
	pos(0) = pos_landing_x;
	pos(1) = pos_landing_y;
	pos(2) = -0.8f;

	PositionCtrl::set_position_command(pos);
	PositionCtrl::update();

	if (_landed)
	{
		return true;
	}

	//orb_check(_ground_contact_sub, &updated);

	/*if (updated) {

		vehicle_land_detected_s landing;

		orb_copy(ORB_ID(vehicle_land_detected), _ground_contact_sub, &landing);
		
		if (landing.ground_contact)
		{
			PX4_INFO("WE HAVE LANDED");
			_landed = true;
		}
	}
	else
	{
		return false; // return true when landed
	}*/
}