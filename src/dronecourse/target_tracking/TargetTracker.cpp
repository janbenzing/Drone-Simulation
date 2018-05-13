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

	// ----------------------------------------------
	// TODO Check validity of parameter handles
	// ----------------------------------------------
	for (int i = 0; i < M; ++i)
	{
		if (_p_kal_sys_noise[i] == PARAM_INVALID)
	 	{
	 		PX4_INFO("WRONG PARAMETER");
	 	}

	}
	// ----------------------------------
	// TODO initialize kalman filter
	// ----------------------------------
	matrix::SquareMatrix<float,M> F;
	F.setZero();
	F(0,3) = 1;
	F(1,4) = 1;
	F(2,5) = 1;

	matrix::Matrix<float,N,M> H;
	H.setZero();
	H(0,0) = 1;
	H(1,1) = 1;
	H(2,2) = 1;

	matrix::Vector<float,M> x0;

	x0(0) = 0;
	x0(1) = 0;
	x0(2) = 0;
	x0(3) = 0;
	x0(4) = 0;
	x0(5) = 0;
	
	matrix::Vector<float, M> p0;

	for (int i = 0; i < M; ++i)
	{
		_w(i) = 0.5;
		p0(i) = 0.5;
	}

	Kalman.init(F, _w, H, x0, p0, dt);

	// ----------------------------------
	// TODO subscribe to uORB topics:
	//  target_position_ned
	// ----------------------------------
	_target_position_ned_sub = orb_subscribe(ORB_ID(target_position_ned));

	// ----------------------------------------------------
	// TODO initialize the handle of your uORB publication
	//      of target_position_ned_filtered to 'nullptr'
	// ----------------------------------------------------
	_target_position_ned_filtered_pub = nullptr;

}

void TargetTracker::update()
{
	// ------------------------------------------------------------
	// TODO call update_parameters() and execute the prediction of
	//      the Kalman filter
	// ------------------------------------------------------------

	bool new_measure = false;
	update_parameters();

	KalmanFilter::predict();
	// ------------------------------------------------------------
	// TODO check if we have a new target_position_ned message
	// and set new_measure accordingly
	// ------------------------------------------------------------
	orb_check(_target_position_ned_sub, &new_measure);

	if (new_measure) {
		// --------------------------------------------------------------------
		// TODO copy message content to a local variable
		// --------------------------------------------------------------------
		matrix::Vector<float, M> measure_val;

		target_position_ned_s target_position;
		orb_copy(ORB_ID(target_position_ned), _target_position_ned_sub, &target_position);
	}
	matrix::Vector<float, N> target_measure;
	target_measure(0) = target_position.x;
	target_measure(1) = target_position.y;
	target_measure(2) = target_position.z;

	matrix::SquareMatrix<float,M> target_measure_cov(target_position.var);
	Kalman.correct(&target_measure,&target_measure_cov);

	// -------------------------------------------------------------------------
	// TODO call publish_filtered_target_position(...) to publish
	//      filtered the filtered target position
	// -------------------------------------------------------------------------
	publish_filtered_target_position(Kalman.get_state_estimate(), Kalman.get_state_variances());
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
			Kalman.set_system_noise(&_w);
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
	target_position_ned_s target_positin_ned_filt;

	target_positin_ned_filt.target_id = 0;

	target_positin_ned_filt.x = pos_vel(0);
	target_positin_ned_filt.y = pos_vel(1);
	target_positin_ned_filt.z = pos_vel(2);

	float var_int[9] = {variance(0), 0, 0,
					0, variance(1), 0,
					0, 0, variance(2)};

	for (int i = 0; i < M*M; ++i)
	{
		target_positin_ned_filt.var_int[i] = 0;
	}

	for (int i = 0; i < M; ++i)
	{
		target_positin_ned_filt.var_int[i] = variance(i);
	}

	// -------------------------------------------------------------------------------------------
	// TODO publish your target_position_ned_s message
	// -------------------------------------------------------------------------------------------
	orb_publish(ORB_ID(target_position_ned_filtered), _target_position_ned_filtered_pub, &target_positin_ned_filt);
}
