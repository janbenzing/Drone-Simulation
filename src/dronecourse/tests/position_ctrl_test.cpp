/**
 * @file target_detection_test.cpp
 * Target detection unit test.
 * @author Arthur Gay <arthur.gay@epfl.ch>
 */

#include "position_ctrl_test.h"
#include "utils.h"
#include "../controllers/PositionCtrl.hpp"
#include "../controllers/GimbalCtrl.hpp"
#include <unit_test/unit_test.h>
#include <uORB/topics/dronecourse_velocity_setpoint.h>
#include <uORB/topics/vehicle_local_position.h>

#define FLOAT_PRECISION 5

class PositionCtrlTest : public UnitTest
{
public:
	PositionCtrlTest();
	virtual ~PositionCtrlTest();

	virtual bool run_tests();

private:
	bool current_position_update_test();
	bool position_command_test();
	bool target_vector_test();
	bool vel_command_test();
	bool goal_reached_test();
	bool goal_not_reached_test();
	bool compute_position_error_test();
	bool compute_velocity_command_test();
};

PositionCtrlTest::PositionCtrlTest() {
}

PositionCtrlTest::~PositionCtrlTest() {
}

bool PositionCtrlTest::current_position_update_test() {
	GimbalCtrl gimbal_ctrl;
	PositionCtrl position_ctrl(gimbal_ctrl);
	matrix::Vector3f expected(-1.001, 78.12, -41.785);

	vehicle_local_position_s position_msg;
	position_msg.x = expected(0);
	position_msg.y = expected(1);
	position_msg.z = expected(2);

	orb_advert_t h = orb_advertise(ORB_ID(vehicle_local_position), &position_msg);

	position_ctrl.update();
	matrix::Vector3f curr_pos = position_ctrl.get_current_position();

	orb_unadvertise(h);

	ut_compare_float("current_position.x", curr_pos(0), expected(0), FLOAT_PRECISION);
	ut_compare_float("current_position.y", curr_pos(1), expected(1), FLOAT_PRECISION);
	ut_compare_float("current_position.z", curr_pos(2), expected(2), FLOAT_PRECISION);

	return true;
}

bool PositionCtrlTest::position_command_test() {
	GimbalCtrl gimbal_ctrl;
	PositionCtrl position_ctrl(gimbal_ctrl);
	matrix::Vector3f expected(-1.001, 78.12, -41.785);

	position_ctrl.set_position_command(expected);
	matrix::Vector3f goal_position = position_ctrl.get_goal_position();

	ut_compare_float("goal_position.x", goal_position(0), expected(0), FLOAT_PRECISION);
	ut_compare_float("goal_position.y", goal_position(1), expected(1), FLOAT_PRECISION);
	ut_compare_float("goal_position.z", goal_position(2), expected(2), FLOAT_PRECISION);

	return true;
}

bool PositionCtrlTest::target_vector_test() {
	GimbalCtrl gimbal_ctrl;
	PositionCtrl position_ctrl(gimbal_ctrl);

	matrix::Vector3f goal_position(1, -1, -1);
	matrix::Vector3f expected(2, -2, 0);

	vehicle_local_position_s position_msg;
	position_msg.x = -1;
	position_msg.y = 1;
	position_msg.z = -1;

	orb_advert_t h = orb_advertise(ORB_ID(vehicle_local_position), &position_msg);

	position_ctrl.set_position_command(goal_position);
	position_ctrl.update();
	matrix::Vector3f target_vector = position_ctrl.get_position_error();

	orb_unadvertise(h);

	compare_vectors("target_vector", expected, target_vector, 3, FLOAT_PRECISION);

	return true;
}

bool PositionCtrlTest::vel_command_test() {
	GimbalCtrl gimbal_ctrl;
	PositionCtrl position_ctrl(gimbal_ctrl);

	matrix::Vector3f goal_position(1, -1, -1);
	matrix::Vector3f expected(16, -16, 0);
	float pos_gain = 8.f;

	param_t pos_gain_param = param_find("POS_GAIN");
	ut_assert("POS_GAIN param", pos_gain_param != PARAM_INVALID);
	param_set(pos_gain_param, &pos_gain);

	vehicle_local_position_s position_msg;
	position_msg.x = -1;
	position_msg.y = 1;
	position_msg.z = -1;

	int sub = orb_subscribe(ORB_ID(dronecourse_velocity_setpoint));
	orb_advert_t h = orb_advertise(ORB_ID(vehicle_local_position), &position_msg);

	position_ctrl.set_position_command(goal_position);
	position_ctrl.update();

	orb_unadvertise(h);

	/* Check topic dronecourse_velocity_setpoint */
	bool updated = false;
	ut_assert("orb_check", orb_check(sub, &updated) == OK);
	ut_assert("updated", updated);

	/* Read topic dronecourse_velocity_setpoint */
	dronecourse_velocity_setpoint_s dc_vel;
	ut_assert("orb_copy", orb_copy(ORB_ID(dronecourse_velocity_setpoint), sub, &dc_vel) == OK);
	ut_compare_float("velocity.vx", dc_vel.vx, expected(0), FLOAT_PRECISION);
	ut_compare_float("velocity.vy", dc_vel.vy, expected(1), FLOAT_PRECISION);
	ut_compare_float("velocity.vz", dc_vel.vz, expected(2), FLOAT_PRECISION);

	return true;
}

bool PositionCtrlTest::goal_reached_test() {
	GimbalCtrl gimbal_ctrl;
	PositionCtrl position_ctrl(gimbal_ctrl);

	// radius should be smaller than the acceptance radius
	float very_small_radius = 0.0005;
	matrix::Vector3f goal_position(1.8, -1.0001, -5.246);

	vehicle_local_position_s position_msg;
	position_msg.x = goal_position(0) + very_small_radius;
	position_msg.y = goal_position(1) - very_small_radius;
	position_msg.z = goal_position(2) + very_small_radius;

	orb_advert_t h = orb_advertise(ORB_ID(vehicle_local_position), &position_msg);
	position_ctrl.set_position_command(goal_position);
	position_ctrl.update();

	orb_unadvertise(h);

	ut_assert_true(position_ctrl.is_goal_reached());

	return true;
}

bool PositionCtrlTest::goal_not_reached_test() {
	GimbalCtrl gimbal_ctrl;
	PositionCtrl position_ctrl(gimbal_ctrl);

	// radius should be greated than the acceptance radius
	float very_small_radius = 15;
	matrix::Vector3f goal_position(1.8, -1.0001, -5.246);

	vehicle_local_position_s position_msg;
	position_msg.x = goal_position(0) + very_small_radius;
	position_msg.y = goal_position(1) - very_small_radius;
	position_msg.z = goal_position(2) + very_small_radius;

	orb_advert_t h = orb_advertise(ORB_ID(vehicle_local_position), &position_msg);
	position_ctrl.set_position_command(goal_position);
	position_ctrl.update();

	orb_unadvertise(h);

	ut_assert_false(position_ctrl.is_goal_reached());

	return true;
}

bool PositionCtrlTest::compute_position_error_test()
{
	GimbalCtrl gimbal_ctrl;
	PositionCtrl position_ctrl(gimbal_ctrl);

	matrix::Vector3f current_position(-96.28f, -72.49f, -80.68f);
	matrix::Vector3f goal_position(77.45f, -75.65f, -88.83f);

	matrix::Vector3f expected(173.73f, -3.16f, -8.15f);

	matrix::Vector3f result = position_ctrl.compute_position_error(goal_position, current_position);

	compare_vectors("Position error", expected, result, 3, FLOAT_PRECISION);

	return true;
}

bool PositionCtrlTest::compute_velocity_command_test()
{
	GimbalCtrl gimbal_ctrl;
	PositionCtrl position_ctrl(gimbal_ctrl);

	matrix::Vector3f error(-96.28f, -72.49f, -80.68f);
	float gain = 0.91f;

	matrix::Vector3f expected(-87.61481f, -65.9659f, -73.4188);

	matrix::Vector3f result = position_ctrl.compute_velocity_command(error, gain);

	compare_vectors("Velocity command", expected, result, 3, FLOAT_PRECISION);

	return true;
}

bool PositionCtrlTest::run_tests()
{
	ut_run_test(current_position_update_test);
	ut_run_test(position_command_test);
	ut_run_test(target_vector_test);
	ut_run_test(vel_command_test);
	ut_run_test(goal_reached_test);
	ut_run_test(goal_not_reached_test);
	ut_run_test(compute_position_error_test);
	ut_run_test(compute_velocity_command_test);
	return (_tests_failed == 0);
}

ut_declare_test(position_ctrl_test, PositionCtrlTest)
