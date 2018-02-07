/**
 * @file TargetDetector.hpp
 * Class to detect moving target
 * Designed for the LIS-EPFL DroneCourse
 *
 * @author Basil Huber LIS <basil.huber@gmail.com>
 */

#pragma once
#include <px4_posix.h>
#include <uORB/Subscription.hpp>
#include <matrix/math.hpp>

#define DEFAULT_HFOV 1.0f
#define DEFAULT_IMAGE_WIDTH 640
#define DEFAULT_IMAGE_HEIGHT 480

class TargetDetector
{

public:
	TargetDetector(const float hfov_default = DEFAULT_HFOV,
	               const int image_width = DEFAULT_IMAGE_WIDTH,
	               const int image_height = DEFAULT_IMAGE_HEIGHT);

	~TargetDetector();

	void update();

	/** Compute the focal length
	 *  @param image_height
	 *  @param field of view
	 *  @return computed focal length
	 */
	float compute_focal_length(
		const int image_height,
		const float hfov);

	/** Compute the centered image coordinates (x, y)
	 *  @param  target_pos 	target location in image coordinates (u, v)
	 *  @param  image_width image horizontal size
	 *  @param  image_height image vertical size
	 *  @return target location in centered image coordinates (x, y)
	 */
	matrix::Vector2f compute_centered_image_coordinates(
		const matrix::Vector2f& _target_pos,
		const int image_width,
		const int image_height);

	/** Compute the scale s
	 *	@param centered_image_coordinates 	target location (x, y)
	 *  @param distance						distance from camera to target d
	 *  @param focal_length 				the focal length f
	 *  @return the scale s
	 */
	float compute_scale(
		const matrix::Vector2f& centered_image_coordinates,
		const float distance,
		const float focal_length);

	/** Find the target location in image frame.
	 *  @param  target_pos_image 	 target position in centered image coordinates (x, y)
	 *  @param  focal_length the focal length f
	 *  @param  scale        the scale s
	 *  @return target location in camera coordinates (X, Y, Z)
	 */
	matrix::Vector3f compute_target_position_image_frame(
		const matrix::Vector2f& target_pos_image,
		const float focal_length,
		const float scale);

	/** Compute covariance matrix
	 *  @param target_pos_image target position in centered image coordinates (x, y)
	 *  @param focal_length f
	 *  @param scale s
	 *  @param target_pos provides the variance measurements
	 *  @return covariance matrix
	 */
	matrix::SquareMatrix<float, 3>  compute_covariance_image_frame(
	const matrix::Vector2f& target_pos_image,
	const float distance,
	const float focal_length,
	const float scale,
	const float var_u,
	const float var_v,
	const float var_d);

	/** Compute a rotation matrix to convert from camera frame to gimbal frame
	 *  @return the computed rotation
	 */
	matrix::Dcm<float> compute_rotation_camera_to_gimbal();

	/** Compute a rotation matrix to convert from gimbal frame to the drone's body frame using the gimbal angles/
	 *  @param pitch angle of the gimble
	 *  @param yaw angle of the gimble
	 *  @return the computed rotation matrix
	 */
	matrix::Dcm<float> compute_rotation_gimbal_to_drone(
		const float pitch,
		const float yaw);

	/** Compute the rotation matrix from image frame to local frame.
	 *  @param image_rot rotation matrix from image frame to camera NED frame
	 *  @param gimbal_rot rotation matrix from camera NED frame to drone's body
	 *  @param att_vehicle rotation matrix
	 *  @return the computed rotation matrix
	 */
	matrix::Dcm<float> compute_rotation_matrix(
		const matrix::Dcm<float>& image_rot,
		const matrix::Dcm<float>& gimbal_rot,
		const matrix::Dcm<float>& att_vehicle);

	/** Compute the target position in the local frame.
	 *  @param total_rot the rotation matrix from image frame to local frame
	 *  @param target_pos the position in image frame
	 *  @param pos_vehicle the position of the drone
	 *  @return target position in local frame
	 */
	matrix::Vector3f compute_target_position_local_frame(
		const matrix::Dcm<float>& total_rot,
		const matrix::Vector3f& target_pos,
		const matrix::Vector3f& pos_vehicle);

	/** Convert covariance matrix from image frame to local frame
	 *  @param var_if covariance matrix in image frame
	 *  @param total_rot rotation matrix from image frame to local frame
	 *  @return covariance matrix in local frame
	 */
	matrix::SquareMatrix<float, 3> compute_covariance_local_frame(
		const matrix::SquareMatrix<float, 3>& var_if,
		const matrix::Dcm<float>& total_rot);

	/** Field of view in radians */
	const float HFOV;
	/** Image horizontal size in pixels */
	const int IMAGE_WIDTH;
	/** Image vertical size in pixels */
	const int IMAGE_HEIGHT;

private:
	void update_subscriptions();

	int _attitude_sub;
	int _position_sub;
	int _target_position_image_sub;

	// uORB publications
	orb_advert_t   _target_position_pub;

	// vehicle attitude and position (from uORB)
	matrix::Dcm<float> _att_vehicle;
	matrix::Vector3f          _pos_vehicle;
};
