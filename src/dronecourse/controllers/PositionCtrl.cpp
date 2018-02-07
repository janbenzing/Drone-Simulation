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
#include <uORB/Subscription.hpp>

PositionCtrl::PositionCtrl(GimbalCtrl &gimbal) :
	BaseCtrl(gimbal),
	_goal_pos(0.0f, 0.0f, 0.0f),
	_current_pos(0.0f, 0.0f, 0.0f)
{
	// --------------------------------------------------------------------------
	// TODO: Add uORB subscription to drone's estimated position
	// --------------------------------------------------------------------------

	// --------------------------------------------------------------------------
	// TODO: Read gain and acceptance radius parameters
	// --------------------------------------------------------------------------
}

void PositionCtrl::update()
{
	update_subscriptions();
	update_parameters();

	print_current_position();
	_position_error = compute_position_error(_goal_pos, _current_pos);
	matrix::Vector3f velocity_command = compute_velocity_command(_position_error, get_position_gain());

	send_velocity_command(velocity_command);
}

void PositionCtrl::print_current_position()
{
	// --------------------------------------------------------------------------
	// TODO: Print current position to console to verify working subscription
	// --------------------------------------------------------------------------
}

matrix::Vector3f PositionCtrl::compute_position_error(matrix::Vector3f goal_pos, matrix::Vector3f current_pos)
{
	// --------------------------------------------------------------------------
	// TODO: Calculate the target vector (vector from drone to goal position)
	// --------------------------------------------------------------------------
	return matrix::Vector3f(0, 0, 0);
}

matrix::Vector3f PositionCtrl::compute_velocity_command(matrix::Vector3f position_error, float position_gain)
{
	// --------------------------------------------------------------------------
	// TODO: Calculate velocity command using proportional gain
	// --------------------------------------------------------------------------
	return matrix::Vector3f(0, 0, 0);
}

bool PositionCtrl::is_goal_reached()
{
	// --------------------------------------------------------------------------
	// TODO: Implement the decision when goal is reached  
	// --------------------------------------------------------------------------
	return true;
}

void PositionCtrl::update_subscriptions()
{
	bool updated;

	// --------------------------------------------------------------------------
	// TODO: Check if drone's local position topic has been updated
	// --------------------------------------------------------------------------

	if (updated) {

		// ------------------------------------------------------------------------
		// TODO: Update current position member variable with new position data
		// ------------------------------------------------------------------------
	}
}

void PositionCtrl::update_parameters()
{
	// --------------------------------------------------------------------------
	// TODO: Get new parameter values and update corresponding member variables
	// --------------------------------------------------------------------------
}
