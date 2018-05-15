/**
 * @file GimbalCtrl.cpp
 * Class to send to control camera gimbal
 * Designed for the LIS-EPFL DroneCourse
 *
 * @author Basil Huber LIS <basil.huber@gmail.com>
 * @author Arthur Gay <arthur.gay@epfl.ch>
 */

#include "GimbalCtrl.hpp"
#include <uORB/topics/gimbal_command.h>
#include <uORB/topics/target_position_ned_filtered.h>
#include <uORB/topics/vehicle_attitude.h>
#include <drivers/drv_hrt.h>

#include <matrix/math.hpp>
#include <matrix/Euler.hpp>

#include <iostream>

GimbalCtrl::GimbalCtrl()
{
	// --------------------------------------------------
	// TODO subscribe to uORB messages:
	//      target_position_ned_filtered,
	//      vehicle_attitude and vehicle_local_position
	// --------------------------------------------------
	_target_position_ned_filtered_sub = orb_subscribe(ORB_ID(target_position_ned_filtered));
	_vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	_local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));


	// set publishing handle for gimbal_command to nullptr
	_gimbal_command_pub = nullptr;

	// Set inital angles downward facing
	set_command(-M_PI / 2, 0.0f);
}

GimbalCtrl::~GimbalCtrl()
{
	// --------------------------------------------------
	// TODO unsubscribe from uORB messages:
	//      target_position_ned_filtered,
	//      vehicle_attitude and vehicle_local_position
	// --------------------------------------------------
	orb_unsubscribe(_target_position_ned_filtered_sub);
	orb_unsubscribe(_vehicle_attitude_sub);
	orb_unsubscribe(_local_pos_sub);

	// Unadvertise
	if(_gimbal_command_pub != nullptr) {
		orb_unadvertise(_gimbal_command_pub);
		_gimbal_command_pub = nullptr;
	}

}

void GimbalCtrl::set_command(float pitch, float yaw)
{
	_mode = MODE::MANUAL;

	// Limit pitch
	if (pitch > 0.0f) {
		pitch = 0.0f;
	} else if (pitch < (float)(- M_PI)) {
		pitch = - M_PI;
	}

	publish_gimbal_command(pitch, yaw);
}

void GimbalCtrl::update()
{
	if (_mode == MODE::AUTOMATIC) {
		// target position in local frame
		matrix::Vector3f target_pos_lf;

		// drone's attitude
		matrix::Quaternion<float> att_vehicle;

		// drone's position
		matrix::Vector3f pos_vehicle;

		// ------------------------------------------------
		// TODO set target position from uORB
		// ------------------------------------------------
		orb_copy(ORB_ID(target_position_ned_filtered), _target_position_ned_filtered_sub, &target_pos_lf);

		// ------------------------------------------------
		// TODO set drone's attitude from uORB
		// ------------------------------------------------
		orb_copy(ORB_ID(vehicle_attitude), _vehicle_attitude_sub, &att_vehicle);

		// ------------------------------------------------
		// TODO set drone position from uORB
		// ------------------------------------------------
		orb_copy(ORB_ID(vehicle_local_position), _local_pos_sub, &pos_vehicle);

		// create variable for target direction
		const matrix::Vector3f target_dir = compute_target_direction(target_pos_lf, pos_vehicle);

		// create variables for drone's Z and Y axis in local frame
		const matrix::Vector3f down_lf = compute_down_axis(att_vehicle);
		const matrix::Vector3f east_lf = compute_east_axis(att_vehicle);

		// create variable for normal vector of auxiliary plane
		// containing target direction and drone's Z axis in local frame
		const matrix::Vector3f n = compute_normal_vector(target_dir, down_lf);

		// create variable for yaw and pitch
		float yaw = compute_yaw(target_dir, n, down_lf, east_lf);
		float pitch = compute_pitch(target_dir, n, down_lf, east_lf);

		// publish uORB message
		publish_gimbal_command(pitch, yaw);
	}
}

matrix::Vector3f GimbalCtrl::compute_target_direction(const matrix::Vector3f target_pos_ned, const matrix::Vector3f pos_vehicle)
{
	// ------------------------------------------------
	// TODO compute target direction
	//      i.e., vector from drone to target
	// ------------------------------------------------
	matrix::Vector3f T;

	T = target_pos_ned - pos_vehicle;
	T.normalize();

	return T;
}

matrix::Vector3f GimbalCtrl::compute_down_axis(matrix::Quaternion<float> attitude)
{
	// ------------------------------------------------
	// TODO compute drone's Z axis in local frame
	// ------------------------------------------------
	//matrix::Dcm<float> Z_local_frame_int(attitude.inversed());
	matrix::Vector3f Z(0,0,1);

	//matrix::Vector3f Z_local_frame;

	//Z_local_frame = Z_local_frame_int*Z;

	return attitude.conjugate_inversed(Z);
}

matrix::Vector3f GimbalCtrl::compute_east_axis(matrix::Quaternion<float> attitude)
{
	// ------------------------------------------------
	// TODO compute drone's Y axis in local frame
	// ------------------------------------------------
	//matrix::Dcm<float> Y_local_frame_int(attitude.inversed());
	matrix::Vector3f Y(0,1,0);

	//matrix::Vector3f Y_local_frame;

	//Y_local_frame = Y_local_frame_int*Y;

	//return Y_local_frame;
	return attitude.conjugate_inversed(Y);
}

matrix::Vector3f GimbalCtrl::compute_normal_vector(const matrix::Vector3f target_direction, const matrix::Vector3f down_lf)
{
	// ---------------------------------------------------
	// TODO find normal vector of auxiliary plane
	//      containing target direction and
	//      drone's Z axis in local frame
	// ---------------------------------------------------
	matrix::Vector3f normal_V;

	normal_V(0) = target_direction(1)*down_lf(2) - target_direction(2)*down_lf(1);
	normal_V(1) = target_direction(2)*down_lf(0) - target_direction(0)*down_lf(2);
	normal_V(2) = target_direction(0)*down_lf(1) - target_direction(1)*down_lf(0);

	return normal_V;
}

float GimbalCtrl::compute_yaw(matrix::Vector3f target_direction,
	matrix::Vector3f n,
	matrix::Vector3f down_lf,
	matrix::Vector3f east_lf)
{
	// ---------------------------------------------------------------------
	// TODO find desired yaw angle as angle between
	//      the plane's normal vector and the drone's Y axis in local frame
	// ---------------------------------------------------------------------
	float Y;

	Y = acos((east_lf*n)/(east_lf.norm()*n.norm()));

	return Y;
}

float GimbalCtrl::compute_pitch(matrix::Vector3f target_direction,
	matrix::Vector3f n,
	matrix::Vector3f down_lf,
	matrix::Vector3f east_lf)
{
	// ----------------------------------------------------------
	// TODO find desired pitch angle as angle between
	//      drone's Z axis in local frame and target direction
	// ----------------------------------------------------------

	float P;

	P = M_PI/2 - acos((down_lf*target_direction)/(down_lf.norm()*target_direction.norm()));

	return P;
}

void GimbalCtrl::publish_gimbal_command(float pitch, float yaw)
{
	// --------------------------------------------------------
	// TODO publish gimbal_command over uORB
	// --------------------------------------------------------
	gimbal_command_s gimbal_command_msg;
	gimbal_command_msg.timestamp = hrt_absolute_time();
	gimbal_command_msg.pitch = pitch;
	gimbal_command_msg.yaw = yaw;
	int instance;
	orb_publish_auto(ORB_ID(gimbal_command), &_gimbal_command_pub, &gimbal_command_msg, &instance, ORB_PRIO_HIGH);
}
