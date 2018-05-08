/**
 * @file TargetTracker.hpp
 * Class to track moving target
 * Designed for the LIS-EPFL DroneCourse
 *
 * @author Basil Huber LIS <basil.huber@gmail.com>
 */

#include "TargetTracker.hpp"
#include <uORB/topics/target_position_ned.h>
#include <uORB/topics/target_position_ned_filtered.h>
#include <drivers/drv_hrt.h>
#include <float.h>

TargetTracker::TargetTracker(float dt)
{
	// ----------------------------------------------
	// TODO set up the arrays of parameter handles
	//      _p_kal_sys_noise
	// ----------------------------------------------
	_p_kal_sys_noise[0] = param_find("xNoise");
	_p_kal_sys_noise[1] = param_find("yNoise");
	_p_kal_sys_noise[2] = param_find("zNoise");
	_p_kal_sys_noise[3] = param_find("vxNoise");
	_p_kal_sys_noise[4] = param_find("vyNoise");
	_p_kal_sys_noise[5] = param_find("vzNoise");
	
	for (int i = 0; i < M; ++i)
	{
		if (_p_kal_sys_noise[i] == PARAM_INVALID)
	 	{
	 		PX4_INFO("WRONG PARAMETER");
	 	}

	}
	

	// ----------------------------------------------
	// TODO Check validity of parameter handles
	// ----------------------------------------------

	// ----------------------------------
	// TODO initialize kalman filter
	// ----------------------------------

	// ----------------------------------
	// TODO subscribe to uORB topics:
	//  target_position_ned
	// ----------------------------------

	// ----------------------------------------------------
	// TODO initialize the handle of your uORB publication
	//      of target_position_ned_filtered to 'nullptr'
	// ----------------------------------------------------
}

void TargetTracker::update()
{
	// ------------------------------------------------------------
	// TODO call update_parameters() and execute the prediction of
	//      the Kalman filter
	// ------------------------------------------------------------

	bool new_measure = false;

	// ------------------------------------------------------------
	// TODO check if we have a new target_position_ned message
	// and set new_measure accordingly
	// ------------------------------------------------------------

	if (new_measure) {
		// --------------------------------------------------------------------
		// TODO copy message content to a local variable
		// --------------------------------------------------------------------
	}

	// -------------------------------------------------------------------------
	// TODO call publish_filtered_target_position(...) to publish
	//      filtered the filtered target position
	// -------------------------------------------------------------------------
}

void TargetTracker::update_parameters()
{
	// ---------------------------------------------------
	// TODO get system noise parameter values and find
	// out if they have changed
	// ---------------------------------------------------
	float sys_noise_param[M];

	for (int i = 0; i < M; ++i)
	{
		param_get(_p_kal_sys_noise[0], &sys_noise_param);
	}

	// ---------------------------------------------------
	// TODO if system noise parameter values have changed,
	// updated system noise of Kalman
	// ---------------------------------------------------
	for (int i = 0; i < M; ++i)
	{
		if (fabsf(_p_kal_sys_noise[i] - _w[i]) > FLT_EPSILON)
		{
			_w[i] = sys_noise_param[i];
			KalmanFilter::set_system_noise(&_w);
		}
	}
}

void TargetTracker::publish_filtered_target_position(const matrix::Vector<float, 6> &pos_vel,
		const matrix::Vector<float, 6> &variance)
{
	// -------------------------------------------------------------------------------------------
	// TODO create local variable of type struct target_position_ned_filtered_s
	//      set all fields to 0 and then fill fields
	// -------------------------------------------------------------------------------------------

	// -------------------------------------------------------------------------------------------
	// TODO publish your target_position_ned_s message
	// -------------------------------------------------------------------------------------------
}
