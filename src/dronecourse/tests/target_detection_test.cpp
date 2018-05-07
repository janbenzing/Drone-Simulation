/**
 * @file target_detection_test.cpp
 * Target detection unit test.
 * @author Arthur Gay <arthur.gay@epfl.ch>
 */

#include "target_detection_test.h"
#include "../target_detection/TargetDetector.hpp"
#include "utils.h"
#include <uORB/topics/target_position_ned.h>
#include <uORB/topics/target_position_image.h>
#include <uORB/topics/vehicle_attitude.h>

#include <unit_test/unit_test.h>
#include <algorithm>
#include <string>

#define FLOAT_PRECISION 4
#define _HFOV 1.0f
#define _IMAGE_WIDTH 1024
#define _IMAGE_HEIGHT 1980

class TargetDetectionTest : public UnitTest
{
public:
	TargetDetectionTest();
	virtual ~TargetDetectionTest();

	virtual bool run_tests();

private:
	bool constructor_default_value_test();
	bool focal_length_test();
	bool compute_centered_image_coordinates_test();
	bool compute_scale_test();
	bool compute_target_position_image_frame_test();
	bool compute_rotation_camera_to_gimbal_test();
	bool compute_rotation_gimbal_to_drone_test();
	bool compute_rotation_matrix_test();
	bool compute_target_position_local_frame_test();
	bool compute_covariance_image_frame_test();
	bool compute_covariance_local_frame_test();
	bool subscribe_publish_test();
};

TargetDetectionTest::TargetDetectionTest() {
}

TargetDetectionTest::~TargetDetectionTest() {
}

bool TargetDetectionTest::constructor_default_value_test()
{
	TargetDetector detector;

	ut_compare_float("IMAGE_WIDTH",
		detector.IMAGE_WIDTH,
		DEFAULT_IMAGE_WIDTH,
		FLOAT_PRECISION);

	ut_compare_float("IMAGE_HEIGHT",
		detector.IMAGE_HEIGHT,
		DEFAULT_IMAGE_HEIGHT,
		FLOAT_PRECISION);

	ut_compare_float("HFOV",
		detector.HFOV,
		DEFAULT_HFOV,
		FLOAT_PRECISION);

	float expected_hfov = 1;
	float expected_width = 2;
	float expected_height = 3;
	TargetDetector detector2(expected_hfov, expected_width, expected_height);

	ut_compare_float("IMAGE_WIDTH",
		detector2.IMAGE_WIDTH,
		expected_width,
		FLOAT_PRECISION);

	ut_compare_float("IMAGE_HEIGHT",
		detector2.IMAGE_HEIGHT,
		expected_height,
		FLOAT_PRECISION);

	ut_compare_float("HFOV",
		detector2.HFOV,
		expected_hfov,
		FLOAT_PRECISION);

	return true;
}

bool TargetDetectionTest::focal_length_test()
{
	TargetDetector detector(_HFOV, _IMAGE_WIDTH, _IMAGE_HEIGHT);
	ut_compare_float("focal length",
		detector.compute_focal_length(detector.IMAGE_WIDTH, detector.HFOV),
		937.20968f,
		FLOAT_PRECISION);
	return true;
}


bool TargetDetectionTest::compute_centered_image_coordinates_test()
{
	TargetDetector detector(_HFOV, _IMAGE_WIDTH, _IMAGE_HEIGHT);

	const matrix::Vector2f target_pos(31.30,75.75);
	const matrix::Vector2f expected(-480.7, -914.25);

	const matrix::Vector2f result = detector.compute_centered_image_coordinates(target_pos, detector.IMAGE_WIDTH, detector.IMAGE_HEIGHT);

	compare_vector2f("centered_image_coordinates", expected, result, FLOAT_PRECISION);

	return true;
}

bool TargetDetectionTest::compute_scale_test()
{
	TargetDetector detector(_HFOV, _IMAGE_WIDTH, _IMAGE_HEIGHT);

	const matrix::Vector2f target_pos(19.56,98.32);
	const float expected = 0.51651;

	const float result = detector.compute_scale(target_pos, detector.IMAGE_WIDTH, detector.IMAGE_HEIGHT);

	ut_compare_float("scale", expected, result, FLOAT_PRECISION);

	return true;
}

bool TargetDetectionTest::compute_target_position_image_frame_test()
{
	TargetDetector detector(_HFOV, _IMAGE_WIDTH, _IMAGE_HEIGHT);

	const matrix::Vector2f target_pos(-25.24,-64.53);
	const float focal_length = 25.88;
	const float scale = 0.5050;
	const matrix::Vector3f expected(-12.74620,-32.58765,13.06940);

	const matrix::Vector3f result = detector.compute_target_position_image_frame(target_pos, focal_length, scale);

	compare_vector3f("target_position_image_frame", expected, result, FLOAT_PRECISION);

	return true;
}

bool TargetDetectionTest::compute_rotation_camera_to_gimbal_test()
{
	TargetDetector detector(_HFOV, _IMAGE_WIDTH, _IMAGE_HEIGHT);

  const matrix::Vector3f target_pos(-2.0, -4.0, 8.0);
  const matrix::Vector3f expected(8.0, -2.0, -4.0);
  const matrix::Dcm<float> mat = detector.compute_rotation_camera_to_gimbal();
  const matrix::Vector3f result = mat * target_pos;

	compare_vector3f("image_to_gimbal", expected, result, FLOAT_PRECISION);

	return true;
}

bool TargetDetectionTest::compute_rotation_gimbal_to_drone_test()
{
	TargetDetector detector(_HFOV, _IMAGE_WIDTH, _IMAGE_HEIGHT);

	const float pitch = 0.26;
	const float yaw = 0.4305;

	matrix::Dcm<float> expected;
	expected(0, 0) = 0.878214;
	expected(0, 1) = -0.417325;
	expected(0, 2) = 0.233624;
	expected(1, 0) = 0.403299;
	expected(1, 1) = 0.908757;
	expected(1, 2) = 0.107286;
	expected(2, 0) = -0.257081;
	expected(2, 1) = 0.000000;
	expected(2, 2) = 0.966390;

	const matrix::Dcm<float> result = detector.compute_rotation_gimbal_to_drone(pitch, yaw);

	compare_matrix("gimbal_to_drone", expected, result, 3, 3, FLOAT_PRECISION);

	return true;
}

bool TargetDetectionTest::compute_rotation_matrix_test()
{
	TargetDetector detector(_HFOV, _IMAGE_WIDTH, _IMAGE_HEIGHT);

	const matrix::Dcm<float> image_rot(matrix::Euler<float>(11.1f, 22.2f, 33.3f));
	const matrix::Dcm<float> gimbal_rot(matrix::Euler<float>(123.4f, -21.3f, 14.9f));
	const matrix::Dcm<float> att_vehicle(matrix::Euler<float>(15.0f, -30.0f, 45.0f));

	matrix::Dcm<float> expected;
	expected(0, 0) = -0.99366f;
	expected(0, 1) = -0.00579f;
	expected(0, 2) = 0.11160f;
	expected(1, 0) = 0.09430f;
	expected(1, 1) = -0.58019f;
	expected(1, 2) = 0.80900f;
	expected(2, 0) = 0.0601f;
	expected(2, 1) = 0.8145f;
	expected(2, 2) = 0.5771f;

	matrix::Dcm<float> result = detector.compute_rotation_matrix(image_rot, gimbal_rot, att_vehicle);

	compare_matrix("rotation", expected, result, 3, 3, FLOAT_PRECISION);

	return true;
}

bool TargetDetectionTest::compute_target_position_local_frame_test()
{
	TargetDetector detector(_HFOV, _IMAGE_WIDTH, _IMAGE_HEIGHT);

	matrix::Dcm<float> total_rot;
	total_rot(0, 0) = 56.94;
	total_rot(0, 1) = -97.58;
	total_rot(0, 2) = -62.14;
	total_rot(1, 0) = -74.64;
	total_rot(1, 1) = 3.70;
	total_rot(1, 2) = -40.21;
	total_rot(2, 0) = -14.97;
	total_rot(2, 1) = -65.71 ;
	total_rot(2, 2) = 32.77;
	const matrix::Vector3f target_pos(48.06,24.63,-28.22);
	const matrix::Vector3f pos_vehicle(-15.90,-53.05,33.76);

	const matrix::Vector3f expected(2070.832031, -2414.391358, -3228.90503);

	matrix::Vector3f result = detector.compute_target_position_local_frame(total_rot, target_pos, pos_vehicle);

	compare_vector3f("target_position_local_frame", expected, result, FLOAT_PRECISION);

	return true;
}

bool TargetDetectionTest::compute_covariance_image_frame_test()
{
	TargetDetector detector(_HFOV, _IMAGE_WIDTH, _IMAGE_HEIGHT);

	const matrix::Vector2f target_pos_image(-38.22,22.71);
	const float distance = -43.33;
	const float focal_length = 7.340;
	const float scale = 0.8960;
	const float var_u = 2.147;
	const float var_v = 3.014;
	const float var_distance = 8.671;

	matrix::SquareMatrix<float, 3> expected;
	expected.setZero();
	expected(0, 0) = 9.498978;
	expected(1, 1) = 5.164874;
	expected(2, 2) = 0.286767;

	matrix::SquareMatrix<float, 3>  result = detector.compute_covariance_image_frame(
		target_pos_image,
		distance,
		focal_length,
		scale,
		var_u,
		var_v,
		var_distance
	);

	compare_matrix("covariance_image_frame", expected, result, 3, 3, FLOAT_PRECISION);

	return true;
}
bool TargetDetectionTest::compute_covariance_local_frame_test()
{
	TargetDetector detector(_HFOV, _IMAGE_WIDTH, _IMAGE_HEIGHT);

	matrix::SquareMatrix<float, 3> var_if;
	matrix::Dcm<float> total_rot;
	// random initialisation
	for(int i = 0; i < 3; i++) {
		for(int j = 0; j < 3; j++) {
			var_if(i, j) = (float []){30.96,-21.67,45.17,-7.23,92.29,-4.30,-80.78,38.71,66.60}[i * 3 + j];
			total_rot(i, j) = (float []){45.16,-6.68,-74.43,-75.90,22.80,-19.71,-10.60,73.41,73.60}[i * 3 + j];
		}
	}

	matrix::SquareMatrix<float, 3> expected;
	expected(0, 0) = 25.572042;
	expected(0, 1) = -78.261361;
	expected(0, 2) = -34.343003;
	expected(1, 0) = 80.568581;
	expected(1, 1) = 55.808586;
	expected(1, 2) = 30.673658;
	expected(2, 0) = 27.944756;
	expected(2, 1) = 91.127144;
	expected(2, 2) = 108.469414;

	matrix::SquareMatrix<float, 3> result = detector.compute_covariance_local_frame(var_if, total_rot);

	compare_matrix("covariance_local_frame", expected, result, 3, 3, FLOAT_PRECISION);

	return true;
}

bool TargetDetectionTest::subscribe_publish_test()
{
	TargetDetector detector(_HFOV, _IMAGE_WIDTH, _IMAGE_HEIGHT);

	struct vehicle_attitude_s attitude_msg;
	attitude_msg.q[0] = 0.720690;
	attitude_msg.q[1] = 0.005302;
	attitude_msg.q[2] = -0.024437;
	attitude_msg.q[3] = 0.692806;

	struct vehicle_local_position_s pos_msg;
	pos_msg.x = -0.280313;
	pos_msg.y = -0.461916;
	pos_msg.z = -2.329061;

	struct target_position_image_s target_pos;
	target_pos.u = 330;
	target_pos.v = 250;
	target_pos.dist = 44.453678f;
	target_pos.pitch = -3.378654f;
	target_pos.yaw = -1.844727f;
	target_pos.var_u = 16;
	target_pos.var_v = 16;
	target_pos.var_dist = 30;

	int instance;

	orb_advert_t vehicle_attitude_pub = nullptr;
	orb_advert_t vehicle_local_position_pub = nullptr;
	orb_advert_t target_position_image_pub = nullptr;

	orb_publish_auto(ORB_ID(vehicle_attitude), &vehicle_attitude_pub, &attitude_msg, &instance, ORB_PRIO_HIGH);
	orb_publish_auto(ORB_ID(vehicle_local_position), &vehicle_local_position_pub, &pos_msg, &instance, ORB_PRIO_HIGH);
	orb_publish_auto(ORB_ID(target_position_image), &target_position_image_pub, &target_pos, &instance, ORB_PRIO_HIGH);

	int sub = orb_subscribe(ORB_ID(target_position_ned));
	ut_assert("Subscriber to target_position_ned", sub != ERROR);

	detector.update();

	bool ok;
	orb_check(sub, &ok);
	ut_assert("Published", ok);

	struct target_position_ned_s result;
	orb_copy(ORB_ID(target_position_ned), sub, &result);

	float expected_x = -40.8227f;
	float expected_y = 4.7289f;
	float expected_z = 15.1495f;

	ut_compare_float("target_pos_ned.x", expected_x, result.x, FLOAT_PRECISION);
	ut_compare_float("target_pos_ned.y", expected_y, result.y, FLOAT_PRECISION);
	ut_compare_float("target_pos_ned.z", expected_z, result.z, FLOAT_PRECISION);

	orb_unadvertise(vehicle_attitude_pub);
	orb_unadvertise(vehicle_local_position_pub);
	orb_unadvertise(target_position_image_pub);
	orb_unsubscribe(sub);

	return true;
}

bool TargetDetectionTest::run_tests()
{
	ut_run_test(constructor_default_value_test);
	ut_run_test(focal_length_test);
	ut_run_test(compute_centered_image_coordinates_test);
	ut_run_test(compute_scale_test);
	ut_run_test(compute_target_position_image_frame_test);
	ut_run_test(compute_rotation_camera_to_gimbal_test);
	ut_run_test(compute_rotation_gimbal_to_drone_test);
	ut_run_test(compute_rotation_matrix_test);
	ut_run_test(compute_target_position_local_frame_test);
	ut_run_test(compute_covariance_image_frame_test);
	ut_run_test(compute_covariance_local_frame_test);
	ut_run_test(subscribe_publish_test);

	return (_tests_failed == 0);
}

ut_declare_test(target_detection_test, TargetDetectionTest)
