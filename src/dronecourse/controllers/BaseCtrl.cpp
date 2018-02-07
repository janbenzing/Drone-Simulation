/**
 * @file BaseController.cpp
 * Base class for controllers
 * provides send_velocity_command(...) to send commands to mc_pos_control
 * that should be called from update
 * Designed for the LIS-EPFL DroneCourse
 *
 * @author Basil Huber LIS <basil.huber@gmail.com>
 */

#include "BaseCtrl.hpp"
#include <uORB/topics/dronecourse_velocity_setpoint.h>
#include <uORB/topics/vehicle_command.h>
#include <drivers/drv_hrt.h>

BaseCtrl::~BaseCtrl()
{
	orb_unadvertise(_vel_setpoint_pub);
}

void BaseCtrl::send_velocity_command(const matrix::Vector3f &vel_command, float yaw_command)
{
	dronecourse_velocity_setpoint_s vel_setpoint;
	vel_setpoint.vx = vel_command(0);
	vel_setpoint.vy = vel_command(1);
	vel_setpoint.vz = vel_command(2);
	vel_setpoint.yaw_valid = (yaw_command == NAN);
	vel_setpoint.yaw = yaw_command;
	vel_setpoint.timestamp = hrt_absolute_time();

	if (_vel_setpoint_pub == nullptr) {
		_vel_setpoint_pub = orb_advertise(ORB_ID(dronecourse_velocity_setpoint), &vel_setpoint);
	} else {
		orb_publish(ORB_ID(dronecourse_velocity_setpoint), _vel_setpoint_pub, &vel_setpoint);
	}
}

void BaseCtrl::arm()
{
	BaseCtrl::arm_disarm(1);
}

void BaseCtrl::disarm()
{
	BaseCtrl::arm_disarm(0);
}

void BaseCtrl::arm_disarm(int armed)
{
	vehicle_command_s cmd = {};
	cmd.target_system = 1;
	cmd.target_component = 1;

	/* Arm vehicle */
	cmd.command = vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM;
	cmd.param1 = armed;
	cmd.param2 = NAN;
	cmd.param3 = NAN;
	cmd.param4 = NAN;
	cmd.param5 = NAN;
	cmd.param6 = NAN;
	cmd.param7 = NAN;
	orb_advert_t h = orb_advertise_queue(ORB_ID(vehicle_command), &cmd, vehicle_command_s::ORB_QUEUE_LENGTH);

	/* Switch to offboard mode */
	cmd.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
	cmd.param1 = 1; /* base mode is custom */
	cmd.param2 = 6; /* custom main mode is offboard */
	cmd.param3 = NAN; /* custom sub mode */
	cmd.param4 = NAN;
	cmd.param5 = NAN;
	cmd.param6 = NAN;
	cmd.param7 = NAN;
	orb_publish(ORB_ID(vehicle_command), h, &cmd);

	(void)orb_unadvertise(h);

	if (armed) {
		PX4_INFO("System armed");

	} else {
		PX4_INFO("System disarmed");
	}
}
