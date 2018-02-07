/**
 * @file GimbalCtrl.hpp
 * Class to send to control camera gimbal
 * Designed for the LIS-EPFL DroneCourse
 *
 * @author Basil Huber LIS <basil.huber@gmail.com>
 */

#pragma once
#include <px4_posix.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_attitude.h>

class GimbalCtrl
{

public:

	enum class MODE {
		MANUAL,
		AUTOMATIC
	};

	GimbalCtrl();

	~GimbalCtrl() {};

	void setAutomatic() {_mode = MODE::AUTOMATIC;};

	void set_command(float pitch, float yaw);

	void update();

private:
	MODE _mode;

	// -----------------------------------------------------
	// TODO create handles for uORB subscriptions:
	//      target_position_ned_filtered,
	//      vehicle_attitude and vehicle_local_position
	// -----------------------------------------------------

	/** handle for uORB publication: gimbal_command */
	orb_advert_t _gimbal_command_pub;
};


