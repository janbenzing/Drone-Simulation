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
	void update_subscriptions();
	void update_parameters();

	GimbalCtrl _gimbal; 
	int Etat;
	int target_position_net_filtered_sub;

	double yaw = 0;
	double pitch = 0;

	float _target_filtered_x;        
	float _target_filtered_y;           
	float _target_filtered_z;           
	float _target_filtered_vx;          
	float _target_filtered_vy;          
	float _target_filtered_vz;          
	float _target_filtered_var_x;      
	float _target_filtered_var_y;       
	float _target_filtered_var_z;       
	float _target_filtered_var_vx;     
	float _target_filtered_var_vy;      
	float _target_filtered_var_vz;       

	
};


