/**
 * @file TargetFollower.hpp
 * Class to follow moving target
 * Designed for the LIS-EPFL DroneCourse
 *
 * @author Basil Huber LIS <basil.huber@gmail.com>
 */

#pragma once
#include "PositionCtrl.hpp"
#include "GimbalCtrl.hpp"


class TargetFollower : public PositionCtrl
{


public:
	TargetFollower(GimbalCtrl &gimbal);

	~TargetFollower() {;};

	void update();

	/**
	* Check if goal is reached
	* true if target_pos and current_pos and _target_vel and current_vel are close enough
	*
	* @return  True if distance to goal smaller than POS_ACCEPT_RAD
	*/
	virtual bool is_goal_reached() const;

private:
};


