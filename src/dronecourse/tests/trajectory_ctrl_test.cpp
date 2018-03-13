/**
 * @file trajectory_ctrl_test.cpp
 * Trajectory controller unit test.
 * @author Arthur Gay <arthur.gay@epfl.ch>
 */

#include "trajectory_ctrl_test.h"
#include "../controllers/TrajectoryCtrl.hpp"
#include "../controllers/GimbalCtrl.hpp"
#include "utils.h"
#include <unit_test/unit_test.h>
#include <matrix/math.hpp>
#include <uORB/topics/vehicle_local_position.h>

#define FLOAT_PRECISION 5

class TrajectoryCtrlTest : public UnitTest
{
public:
	TrajectoryCtrlTest();
	virtual ~TrajectoryCtrlTest();

	virtual bool run_tests();

private:
	bool goal_reached_test();
};

class MockWaypointNavigator : public WaypointNavigator
{
public:
	int waypoint_copy(int index, matrix::Vector3f *waypoint) override
	{
		*waypoint = WAYPOINTS[index % WAYPOINT_COUNT];
		return index;
	};

	int waypoint_count() override { return WAYPOINT_COUNT; };

	static const int WAYPOINT_COUNT = 3;
	const matrix::Vector3f WAYPOINTS[WAYPOINT_COUNT] = {
		matrix::Vector3f(10, 10, -10),
		matrix::Vector3f(20, 20, -20),
		matrix::Vector3f(30, 30, -30)
	};
};

TrajectoryCtrlTest::TrajectoryCtrlTest() {
}

TrajectoryCtrlTest::~TrajectoryCtrlTest() {
}

bool TrajectoryCtrlTest::goal_reached_test() {
	MockWaypointNavigator navigator;
	GimbalCtrl gimbal_ctrl;
	TrajectoryCtrl trajectory_ctrl(gimbal_ctrl, navigator);
	matrix::Vector3f goal_position;

	/* First waypoint */
	set_local_position_vector3f(matrix::Vector3f(0, 0, 0));
	trajectory_ctrl.update();
	trajectory_ctrl.update();

	goal_position = trajectory_ctrl.get_goal_position();
	compare_vector3f("goal_position", goal_position, navigator.WAYPOINTS[0], FLOAT_PRECISION);
	ut_assert_false(trajectory_ctrl.is_goal_reached());

	/* Second waypoint */
	set_local_position_vector3f(navigator.WAYPOINTS[0]);
	trajectory_ctrl.update();
	trajectory_ctrl.update();

	goal_position = trajectory_ctrl.get_goal_position();
	compare_vector3f("goal_position", goal_position, navigator.WAYPOINTS[1], FLOAT_PRECISION);
	ut_assert_false(trajectory_ctrl.is_goal_reached());

	/* Third waypoint */
	set_local_position_vector3f(navigator.WAYPOINTS[1]);
	trajectory_ctrl.update();
	trajectory_ctrl.update();

	goal_position = trajectory_ctrl.get_goal_position();
	compare_vector3f("goal_position", goal_position, navigator.WAYPOINTS[2], FLOAT_PRECISION);
	ut_assert_false(trajectory_ctrl.is_goal_reached());

	/* Third waypoint */
	set_local_position_vector3f(navigator.WAYPOINTS[2]);
	trajectory_ctrl.update();
	trajectory_ctrl.update();

	ut_assert_true(trajectory_ctrl.is_goal_reached());

	return true;
}

bool TrajectoryCtrlTest::run_tests()
{
	ut_run_test(goal_reached_test);
	return (_tests_failed == 0);
}

ut_declare_test(trajectory_ctrl_test, TrajectoryCtrlTest)
