/**
 * @file gimbal_ctrl_test.cpp
 * Gimbal controller unit test.
 * @author Arthur Gay <arthur.gay@epfl.ch>
 */

#include "gimbal_ctrl_test.h"
#include "../controllers/GimbalCtrl.hpp"
#include "utils.h"
#include <uORB/topics/gimbal_command.h>
#include <uORB/topics/target_position_ned_filtered.h>
#include <uORB/topics/vehicle_attitude.h>

#include <unit_test/unit_test.h>
#include <matrix/math.hpp>

#define FLOAT_PRECISION 5

class GimbalCtrlTest : public UnitTest
{
public:
	GimbalCtrlTest();
	virtual ~GimbalCtrlTest();

	virtual bool run_tests();

private:
	bool set_command_test();
	bool compute_target_direction_test();
	bool compute_down_axis_test();
	bool compute_east_axis_test();
	bool compute_normal_vector_test();
	bool compute_yaw_test();
	bool compute_pitch_test();
	bool update_test();
};

GimbalCtrlTest::GimbalCtrlTest() {
}

GimbalCtrlTest::~GimbalCtrlTest() {
}

bool GimbalCtrlTest::set_command_test()
{
	GimbalCtrl ctrl;
	float expected_pitch = -0.1f;
	float expected_yaw = 0.25f;

	int sub = orb_subscribe(ORB_ID(gimbal_command));
	ut_assert("Subscriber to gimbal_command", sub != ERROR);

	ctrl.set_command(expected_pitch, expected_yaw);

	bool updated = false;
	ut_assert("orb_check", orb_check(sub, &updated) == OK);
	ut_assert("updated", updated);

	gimbal_command_s cmd;
	ut_assert("orb_copy", orb_copy(ORB_ID(gimbal_command), sub, &cmd) == OK);
	ut_compare_float("gimbal.pitch", cmd.pitch, expected_pitch, FLOAT_PRECISION);
	ut_compare_float("gimbal.yaw", cmd.yaw, expected_yaw, FLOAT_PRECISION);

	orb_unsubscribe(sub);

	return true;
}

bool GimbalCtrlTest::compute_target_direction_test()
{
	GimbalCtrl ctrl;
	const matrix::Vector3f target_pos_ned(77.04f, 68.26f, 33.00f);
	const matrix::Vector3f pos_vehicle(46.78f, 80.34f, 98.95f);
	const matrix::Vector3f expected(0.41137f, -0.16422f, -0.89655f);

	const matrix::Vector3f target_direction = ctrl.compute_target_direction(target_pos_ned, pos_vehicle);

	compare_vector3f("target_direction", expected, target_direction, FLOAT_PRECISION);

	return true;
}

bool GimbalCtrlTest::compute_down_axis_test()
{
	GimbalCtrl ctrl;

	const matrix::Quaternion<float> attitude(21.31f, 87.44f, 68.19f, 26.11f);
	const matrix::Vector3f expected(0.55633f, -0.01234f, -0.83087f);

	const matrix::Vector3f down_axis = ctrl.compute_down_axis(attitude);

	compare_vector3f("down_axis", expected, down_axis, FLOAT_PRECISION);

	return true;
}

bool GimbalCtrlTest::compute_east_axis_test()
{
	GimbalCtrl ctrl;

	const matrix::Quaternion<float> attitude(30.82f, 54.20f, 51.22f, 51.52f);
	const matrix::Vector3f expected(0.25930f, -0.22024f, 0.94035f);

	const matrix::Vector3f east_axis = ctrl.compute_east_axis(attitude);

	compare_vector3f("east_axis", expected, east_axis, FLOAT_PRECISION);

	return true;
}

bool GimbalCtrlTest::compute_normal_vector_test()
{
	GimbalCtrl ctrl;

	const matrix::Vector3f target_direction(0.65848f, 0.41734f, 0.62628f);
	const matrix::Vector3f down_lf(0.59149f, 0.22083f, 0.77548f);
	const matrix::Vector3f expected(0.73092f, -0.55291f, -0.40005f);

	const matrix::Vector3f n = ctrl.compute_normal_vector(target_direction, down_lf);

	compare_vector3f("normal_vector", expected, n, FLOAT_PRECISION);

	return true;
}

bool GimbalCtrlTest::compute_yaw_test()
{
	GimbalCtrl ctrl;

	const matrix::Vector3f target_direction(0.47924f, 0.85192f, 0.21108f);
	const matrix::Vector3f n(0.71116f, 0.6901f, 0.13421f);
	const matrix::Vector3f down_lf(0.73738f, 0.42073f, 0.52845f);
	const matrix::Vector3f east_lf(0.18832f, 0.40864f, 0.89306f);

	const float expected = -1.00536f;

	const float yaw = ctrl.compute_yaw(target_direction, n, down_lf, east_lf);

	ut_compare_float("yaw", expected, yaw, FLOAT_PRECISION);

	return true;
}

bool GimbalCtrlTest::compute_pitch_test()
{
	GimbalCtrl ctrl;

	const matrix::Vector3f target_direction(0.58082f, 0.63421f, 0.51032f);
	const matrix::Vector3f n(0.7291f, 0.6747f, 0.1140f);
	const matrix::Vector3f down_lf(0.5441f, 0.5778f, 0.6082f);
	const matrix::Vector3f east_lf(0.3132f, 0.6783f, 0.6645f);

	const float expected = -1.69047f;

	const float pitch = ctrl.compute_pitch(target_direction, n, down_lf, east_lf);

	ut_compare_float("pitch", expected, pitch, FLOAT_PRECISION);

	return true;
}

bool GimbalCtrlTest::update_test()
{
	GimbalCtrl ctrl;

	// Set target position in local frame
	struct target_position_ned_filtered_s target_pos_msg;
	target_pos_msg.x = 32.36f;
	target_pos_msg.y = 54.72f;
	target_pos_msg.z = 91.19f;
	orb_advertise(ORB_ID(target_position_ned_filtered), &target_pos_msg);

	// Set drone's attitude
	struct vehicle_attitude_s vehicle_attitude_msg;
	vehicle_attitude_msg.q[0] = 52.02f;
	vehicle_attitude_msg.q[1] = 32.16f;
	vehicle_attitude_msg.q[2] = 92.73f;
	vehicle_attitude_msg.q[3] = 65.46f;
	orb_advertise(ORB_ID(vehicle_attitude), &vehicle_attitude_msg);

	// Set drone's position
	struct vehicle_local_position_s vehicle_position_msg;
	vehicle_position_msg.x = 53.54f;
	vehicle_position_msg.y = 48.78f;
	vehicle_position_msg.z = 88.53f;
	orb_advertise(ORB_ID(vehicle_local_position), &vehicle_position_msg);

	// Subscribe
	int sub = orb_subscribe(ORB_ID(gimbal_command));
	ut_assert("Subscriber to gimbal_command", sub != ERROR);

	ctrl.setAutomatic();
	ctrl.update();

	// Read from uORB
	bool updated = false;
	ut_assert("orb_check", orb_check(sub, &updated) == OK);
	ut_assert("updated", updated);

	// Asserts
	float expected_pitch = -3.88133;
	float expected_yaw = -2.78622;
	gimbal_command_s cmd;
	ut_assert("orb_copy", orb_copy(ORB_ID(gimbal_command), sub, &cmd) == OK);
	ut_compare_float("gimbal.pitch", cmd.pitch, expected_pitch, FLOAT_PRECISION);
	ut_compare_float("gimbal.yaw", cmd.yaw, expected_yaw, FLOAT_PRECISION);

	orb_unsubscribe(sub);

	return true;
}

bool GimbalCtrlTest::run_tests()
{
	ut_run_test(set_command_test);
	ut_run_test(compute_target_direction_test);
	ut_run_test(compute_down_axis_test);
	ut_run_test(compute_east_axis_test);
	ut_run_test(compute_normal_vector_test);
	ut_run_test(compute_yaw_test);
	ut_run_test(compute_pitch_test);
	ut_run_test(update_test);
	return (_tests_failed == 0);
}

ut_declare_test(gimbal_ctrl_test, GimbalCtrlTest)
