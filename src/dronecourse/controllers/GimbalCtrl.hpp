/**
 * @file GimbalCtrl.hpp
 * Class to send to control camera gimbal
 * Designed for the LIS-EPFL DroneCourse
 *
 * @author Basil Huber LIS <basil.huber@gmail.com>
 * @author Arthur Gay <arthur.gay@epfl.ch>
 */

#pragma once
#include <px4_posix.h>
#include <uORB/Subscription.hpp>
#include <matrix/math.hpp>

class GimbalCtrl
{

public:

	enum class MODE {
		MANUAL,
		AUTOMATIC
	};

	GimbalCtrl();

	~GimbalCtrl();

	void setAutomatic() {_mode = MODE::AUTOMATIC;};

	void set_command(float pitch, float yaw);

	void update();

	/** Compute target direction i.e., vector from drone to target. */
	matrix::Vector3f compute_target_direction(
		matrix::Vector3f target_pos_ned,
		matrix::Vector3f pos_vehicle);

	/** Compute drone's Z axis in local frame */
	matrix::Vector3f compute_down_axis(matrix::Quaternion<float> attitude);

	/** Compute drone's Y axis in local frame */
	matrix::Vector3f compute_east_axis(matrix::Quaternion<float> attitude);

	/** Compute normal vector of auxiliary plane containing target
	 * direction and drone's Z axis in local frame. */
	matrix::Vector3f compute_normal_vector(
		matrix::Vector3f target_direction,
		matrix::Vector3f down_lf);

	/** Compute desired pitch angle as angle between drone's Z axis in
	 * local frame and target direction */
	float compute_pitch(matrix::Vector3f target_direction,
		matrix::Vector3f n,
		matrix::Vector3f east_lf,
		matrix::Vector3f down_lf);

	/** Compute desired yaw angle as angle between the plane's normal
	 * vector and the drone's Y axis in local frame */
	float compute_yaw(matrix::Vector3f target_direction,
		matrix::Vector3f n,
		matrix::Vector3f east_lf,
		matrix::Vector3f down_lf);

private:
	MODE _mode;

	// -----------------------------------------------------
	// TODO create handles for uORB subscriptions:
	//      target_position_ned_filtered,
	//      vehicle_attitude and vehicle_local_position
	// -----------------------------------------------------

	/** handle for uORB publication: gimbal_command */
	orb_advert_t _gimbal_command_pub;

	/** Publish command */
	void publish_gimbal_command(float pitch, float yaw);
};


