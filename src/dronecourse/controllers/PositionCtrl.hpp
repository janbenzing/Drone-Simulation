/**
 * @file PositionCtrl.hpp
 * Class to convert position command to velocity command
 * Designed for the LIS-EPFL DroneCourse
 *
 * @author Basil Huber LIS <basil.huber@gmail.com>
 * @author Fabian Schilling <fabian.schilling@epfl.ch>
 */

#pragma once
#include <px4_posix.h>
#include "BaseCtrl.hpp"

class PositionCtrl : public BaseCtrl
{

public:

	/**
	 * Constructor
	 *
	 * @param gimbal     Gimbal controller to control pitch and yaw of gimbal
	 */
	PositionCtrl(GimbalCtrl &gimbal);

	~PositionCtrl() {};

	/**
	 * Updates the controller
	 * i.e. calculates velocity to reach goal position
	 * and sends it using BaseCtrl::send_velocity_command(...)
	 *
	 * @return  drone position in local frame
	 */
	void update();

	/**
	 * Check if goal is reached
	 * i.e. if we are close enough to waypoint
	 * (acceptance radius define with parameter POS_ACCEPT_RAD)
	 *
	 * @return  True if distance to goal smaller than POS_ACCEPT_RAD
	 */
	virtual bool is_goal_reached();

	/**
	 * Sets goal position of this controller
	 *
	 * @param pos   Goal position in local frame
	 */
	void set_position_command(matrix::Vector3f &pos) {_goal_pos = pos;};

	/**
	 * Returns goal position in local frame
	 * (set by set_position_command);
	 *
	 * @return  drone position in local frame
	 */
	const matrix::Vector3f &get_goal_position() const {return _goal_pos;};

	/**
	 * Returns drone position in local frame
	 * (updated by calling update_subscriptions())
	 *
	 * @return  drone position in local frame
	 */
	const matrix::Vector3f &get_current_position() const {return _current_pos;};

	/**
	 * Returns goal position (set by set_position_command) with respect to the drone position
	 * (vector from the drone to the goal position in local frame)
	 * Updated by update
	 *
	 * @return  goal position in drone centered local frame
	 */
	const matrix::Vector3f &get_position_error() const {return _position_error;};

	/**
	 * Compute error vector between the current position and the goal position
	 * @param goal_pos goal position
	 * @param current_pos current position
	 * @return computed error
	 */
	matrix::Vector3f compute_position_error(matrix::Vector3f goal_pos, matrix::Vector3f current_pos);

	/**
	 * Compute the required velocity vector using the given position error.
	 * @param position_error error vector between current position and the goal position
	 * @param position_gain proportional gain to compute the velocity command
	 * @return computed velocity command
	 */
	matrix::Vector3f compute_velocity_command(matrix::Vector3f position_error, float position_gain);

protected:

	/**
	 * Updates uORB subscriptions:
	 * i.e. updates _current_pos with vehicle_local_position uORB message
	 */
	void update_subscriptions();

	/**
	 * Updates onboard parameters:
	 * i.e. update POS_ACCEPT_RAD
	 */
	void update_parameters();

	/** Return position gain. */
	virtual float get_position_gain() { return _pos_gain; };

	/** Return acceptance radius. */
	virtual float get_position_accept_rad() { return _pos_accept_rad; };

private:

	/** Goal position in local frame */
	matrix::Vector3f _goal_pos;

	/** Current position in local frame
	 * @see update_subscriptions
	 */
	matrix::Vector3f _current_pos;

	/** Vector from drone to goal position */
	matrix::Vector3f _position_error;

	/** uORB subscription to local position topic */
	int _local_pos_sub;

	/** Onboard parameter handle for position controller gain */
	param_t _p_pos_gain;

	// --------------------------------------------------------------------------
	// TODO: Create a member variable for acceptance radius parameter handle
	// --------------------------------------------------------------------------
	param_t _p_accept_rad;
	/**
	 * Onboard parameter value for position controller gain.
	 * You should not use this variable directly in your code, you
	 * should use get_position_gain(). That way, you can override this
	 * parameter by implementing get_position_gain in the child class.
	 */
	float   _pos_gain;

	/**
	 * Onboard parameter value for acceptance radius for goal.
	 * You should not use this variable directly in your code, you
	 * should use get_position_accept_rad(). That way, you can override
	 * this parameter by implementing get_position_accept_rad in the
	 * child class.
	 */
	float   _pos_accept_rad;

	/**
	 * Print the current position vector to standard output
	 */
	void print_current_position();
};
