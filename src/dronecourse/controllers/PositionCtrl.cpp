/**
 * @file PositionCtrl.cpp
 * Class to convert position command to velocity command
 * Designed for the LIS-EPFL DroneCourse
 *
 * @author Basil Huber LIS <basil.huber@gmail.com>
 * @author Arthur Gay <arthur.gay@epfl.ch>
 * @author Fabian Schilling <fabian.schilling@epfl.ch>
 */

#include "PositionCtrl.hpp"
#include <uORB/topics/vehicle_local_position.h>
//#include <uORB/topics/parameter_update.h>
#include <uORB/Subscription.hpp>

PositionCtrl::PositionCtrl(GimbalCtrl &gimbal) :
	BaseCtrl(gimbal),
	_goal_pos(0.0f, 0.0f, 0.0f),
	_current_pos(0.0f, 0.0f, 0.0f)
{
	// --------------------------------------------------------------------------
	// TODO: Add uORB subscription to drone's estimated position
	// --------------------------------------------------------------------------
	_local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	// --------------------------------------------------------------------------
	// TODO: Read gain and acceptance radius parameters
	// --------------------------------------------------------------------------
	update_parameters();

}

void PositionCtrl::update()
{
	update_subscriptions();
	update_parameters();
	is_goal_reached();

	print_current_position();
	_position_error = compute_position_error(_goal_pos, _current_pos);
	matrix::Vector3f velocity_command = compute_velocity_command(_position_error, get_position_gain());
	//matrix::Vector3f velocity_command = compute_velocity_command(_position_error, 0.3);

	send_velocity_command(velocity_command);
}

void PositionCtrl::print_current_position()
{
	// --------------------------------------------------------------------------
	// TODO: Print current position to console to verify working subscription
	// --------------------------------------------------------------------------
	//PX4_INFO("Info message with x = %f", (double)_current_pos(0));
	//PX4_INFO("Info message with y = %f", (double)_current_pos(1));
	//PX4_INFO("Info message with z = %f", (double)_current_pos(2));

}

matrix::Vector3f PositionCtrl::compute_position_error(matrix::Vector3f goal_pos, matrix::Vector3f current_pos)
{
	// --------------------------------------------------------------------------
	// TODO: Calculate the target vector (vector from drone to goal position)
	// --------------------------------------------------------------------------
	matrix::Vector3f _target_vector = goal_pos - current_pos;
	
	return _target_vector;

}

matrix::Vector3f PositionCtrl::compute_velocity_command(matrix::Vector3f position_error, float position_gain)
{
	// --------------------------------------------------------------------------
	// TODO: Calculate velocity command using proportional gain
	// --------------------------------------------------------------------------
	matrix::Vector3f _velo_command = position_error*position_gain;
	return _velo_command;
	//return matrix::Vector3f(0, 0, 0);
}

bool PositionCtrl::is_goal_reached()
{
	// --------------------------------------------------------------------------
	// TODO: Implement the decision when goal is reached  
	// --------------------------------------------------------------------------
	//if (_position_error.norm() < _pos_accept_rad)
	//{
	//	PX4_INFO("GOAL IS REACHED");
	//}
	return true;
}

void PositionCtrl::update_subscriptions()
{
	bool updated;

	// --------------------------------------------------------------------------
	// TODO: Check if drone's local position topic has been updated
	// --------------------------------------------------------------------------
	orb_check(_local_pos_sub, &updated);

	if (updated) {

		// ------------------------------------------------------------------------
		// TODO: Update current position member variable with new position data
		// ------------------------------------------------------------------------
		vehicle_local_position_s local_pos;
		orb_copy(ORB_ID(vehicle_local_position), _local_pos_sub, &local_pos);
		_current_pos(0) = local_pos.x;
		_current_pos(1) = local_pos.y;
		_current_pos(2) = local_pos.z;
	}
}

void PositionCtrl::update_parameters()
{
	// --------------------------------------------------------------------------
	// TODO: Get new parameter values and update corresponding member variables
	// --------------------------------------------------------------------------
	_p_pos_gain = param_find("POS_GAIN");
	
	if (_p_pos_gain == PARAM_INVALID)
	 {
	 	PX4_INFO("WRONG PARAMETER POS GAIN");
	 }

	param_get(_p_pos_gain, &_pos_gain);

	_p_accept_rad = param_find("POS_ACCEPT_RAD");
	
	if (_p_accept_rad == PARAM_INVALID)
	 {
	 	PX4_INFO("WRONG PARAMETER ACCEPT RAD");
	 }

	param_get(_p_accept_rad, &_pos_accept_rad);
}
