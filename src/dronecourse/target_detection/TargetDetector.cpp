/**
 * @file TargetDetector.hpp
 * Class to track moving target
 * Designed for the LIS-EPFL DroneCourse
 *
 * @author Basil Huber LIS <basil.huber@gmail.com>
 */

#include "TargetDetector.hpp"
#include <matrix/math.hpp>
#include <uORB/topics/target_position_ned.h>
#include <uORB/topics/target_position_image.h>
#include <uORB/topics/vehicle_attitude.h>
#include <drivers/drv_hrt.h>

TargetDetector::TargetDetector(const float hfov_default,
	               const int image_width,
	               const int image_height) :
	               HFOV(hfov_default),
	               IMAGE_WIDTH(image_width),
	               IMAGE_HEIGHT(image_height)
{
	// --------------------------------------------------------------------------
	// TODO subscribe to uORB topics:
	//  target_position_image messages, vehicle_attitude, vehicle_local_position
	// --------------------------------------------------------------------------
	_target_position_image_sub = orb_subscribe(ORB_ID(target_position_image));

	//_attitude_sub(orb_subscribe(orb_subscribe(ORB_ID(vehicle_attitude)));

	_local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));

	_vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));

	// --------------------------------------------------------------------------
	// TODO set uORB publishing handle '_target_position_pub' to nullptr
	// --------------------------------------------------------------------------
}

TargetDetector::~TargetDetector()
{
	// --------------------------------------------------------------------------
	// TODO unsubscribe from uORB topics:
	//  target_position_image messages, vehicle_attitude, vehicle_local_position
	// --------------------------------------------------------------------------
}

void TargetDetector::update()
{
	update_subscriptions();

	bool new_measure = false;
	// ------------------------------------------------------------
	// TODO check if we have a new _target_position_image message
	// and set new_measure accordingly
	// ------------------------------------------------------------

	orb_check(_target_position_image_sub, &new_measure);



	if (new_measure) {
		// --------------------------------------------------------------------
		// TODO copy message content to a local variable
		// --------------------------------------------------------------------
		struct target_position_image_s target_pos;

		orb_copy(ORB_ID(target_position_image), _target_position_image_sub, &target_pos);

		// ~~~ Computations ~~~
		/** The target position in the image (u, v) */
		const matrix::Vector2f target_pos_image(target_pos.u, target_pos.v);
		/** The distance from the camera to the target*/
		const float distance = target_pos.dist;

		/** Focal length */
		const float focal_length = compute_focal_length(IMAGE_WIDTH, HFOV);

		/** The target position in centered image coordinates (x, y) */
		const matrix::Vector2f centered_image_coordinates = compute_centered_image_coordinates(target_pos_image, IMAGE_WIDTH, IMAGE_HEIGHT);

		/** The scale s */
		const float scale = compute_scale(centered_image_coordinates, distance, focal_length);

		/** The target location in image frame */
		const matrix::Vector3f target_pos_image_frame = compute_target_position_image_frame(centered_image_coordinates, focal_length, scale);

		/** Rotation matrix to convert from camera frame to gimbal frame from euler angles */
		const matrix::Dcm<float> image_rot = compute_rotation_camera_to_gimbal();

		/** Rotation matrix to convert from gimbal frame to  the drone's body frame using the gimbal angles */
		const matrix::Dcm<float> gimbal_rot = compute_rotation_gimbal_to_drone(target_pos.pitch, target_pos.yaw);

		/** Rotation matrix (dcm) from image frame to local frame using previous rotation matrices and the attitude of the vehicle */
		const matrix::Dcm<float> total_rot = compute_rotation_matrix(image_rot, gimbal_rot, _att_vehicle);

		/** Target position in local frame */
		const matrix::Vector3f target_pos_local_frame = compute_target_position_local_frame(total_rot, target_pos_image_frame, _pos_vehicle);

		/** The covariance matrix */
		const matrix::SquareMatrix<float, 3> var_image_frame = compute_covariance_image_frame(centered_image_coordinates,
			distance, focal_length, scale, target_pos.var_u, target_pos.var_v, target_pos.var_dist);

		/** Convert covariance matrix from image frame to local frame */
		const matrix::SquareMatrix<float, 3> var_local_frame = compute_covariance_local_frame(var_image_frame, total_rot);

		// Publish computed position in local frame
		struct target_position_ned_s target_pos_ned;
		memset(&target_pos_ned, 0, sizeof(target_pos_ned));     // set all fields to 0
		target_pos_ned.x = target_pos_local_frame(0);
		target_pos_ned.y = target_pos_local_frame(1);
		target_pos_ned.z = target_pos_local_frame(2);
		target_pos_ned.timestamp = hrt_absolute_time();
		memcpy(&target_pos_ned.var, var_local_frame.data(), sizeof(target_pos_ned.var));

		int instance;
		orb_publish_auto(ORB_ID(target_position_ned), &_target_position_pub, &target_pos_ned, &instance, ORB_PRIO_HIGH);
	}
}

/** Compute the focal length
 *  @param image_width
 *  @param hfov field of view
 *  @return computed focal length
 */
float TargetDetector::compute_focal_length(const int image_width, const float hfov)
{
	// -----------------------------------------------------------------
	// TODO compute the focal length f
	// -----------------------------------------------------------------
	return (image_width/2)/tan(hfov/2);
}

/** Compute the centered image coordinates (x, y)
 *  @param  target_pos 	target location in image coordinates (u, v)
 *  @param  image_width image horizontal size
 *  @param  image_height image vertical size
 *  @return target location in centered image coordinates (x, y)
 */
matrix::Vector2f TargetDetector::compute_centered_image_coordinates(
	const matrix::Vector2f& target_pos,
	const int image_width,
	const int image_height)
{
	// -----------------------------------------------------------------
	// TODO compute target's position in centered image coordinates
	// -----------------------------------------------------------------
	target_pos_c_image(0) = target_pos(0)-(image_width/2); 
	target_pos_c_image(1) = target_pos(1)-(image_height/2);

	return target_pos_c_image; 
}

/** Compute the scale s
 *	@param centered_image_coordinates 	target location (x, y)
 *  @param distance						distance from camera to target d
 *  @param focal_length 				the focal length f
 *  @return the scale s
 */
float TargetDetector::compute_scale(
	const matrix::Vector2f& centered_image_coordinates,
	const float distance,
	const float focal_length)
{
	// -----------------------------------------------------------------
	// TODO compute the scale s
	// -----------------------------------------------------------------
	return (double)distance/sqrt(centered_image_coordinates(0)*centered_image_coordinates(0)+centered_image_coordinates(1)*centered_image_coordinates(1)+focal_length*focal_length);
}

/** Find the target location in image frame.
 *  @param  target_pos_image 	 target position in centered image coordinates (x, y)
 *  @param  focal_length the focal length f
 *  @param  scale        the scale s
 *  @return target location in camera coordinates (X, Y, Z)
 */
matrix::Vector3f TargetDetector::compute_target_position_image_frame(const matrix::Vector2f& target_pos_image, const float focal_length, const float scale)
{
	// -----------------------------------------------------------------
	// TODO compute target's position in image frame
	// -----------------------------------------------------------------
	matrix::Vector3f target_pos_i_image;

	target_pos_i_image(0) = target_pos_c_image(0)*scale;
	target_pos_i_image(1) = target_pos_c_image(1)*scale;
	target_pos_i_image(2) = focal_length*scale;

	return target_pos_i_image;
}

/** Compute covariance matrix
 *  @param target_pos_image target position in centered image coordinates (x, y)
 *  @param focal_length f
 *  @param scale s
 *  @param target_pos provides the variance measurements
 *  @return covariance matrix
 */
matrix::SquareMatrix<float, 3>  TargetDetector::compute_covariance_image_frame(
	const matrix::Vector2f& target_pos_image,
	const float distance,
	const float focal_length,
	const float scale,
	const float var_u,
	const float var_v,
	const float var_d)
{
	// -----------------------------------------------------------------
	// TODO compute covariance matrix in image frame
	// -----------------------------------------------------------------
	float sigma2_X;
	float sigma2_Y;
	float sigma2_Z;
	float sigma2_s;
	float l;

	l = sqrt(target_pos_image(0)*target_pos_image(0)+target_pos_image(1)*target_pos_image(1)+focal_length*focal_length);

	sigma2_s = (var_d*var_d)/(l*l) + ((distance*distance)/(l*l*l*l*l*l*l)*((target_pos_image(0)*target_pos_image(0))*(var_u*var_u)+(target_pos_image(1)*target_pos_image(1))*(var_v*var_v)));

	sigma2_X =  sigma2_s*(target_pos_image(0)*target_pos_image(0))+(var_u*var_u)*(scale*scale);
	sigma2_Y =  sigma2_s*(target_pos_image(1)*target_pos_image(1))+(var_v*var_v)*(scale*scale);
	sigma2_Z =  sigma2_s*(focal_length*focal_length);

	const float cov_matrix[9] = { sigma2_X, 0.0f, 0.0f,
							0.0f, sigma2_Y, 0.0f,
							0.0f, 0.0f, sigma2_Z};



	return cov_matrix;
}

/** Compute a rotation matrix to convert from camera frame to gimbal frame
 *  @return the computed rotation matrix
 */
matrix::Dcm<float> TargetDetector::compute_rotation_camera_to_gimbal()
{
	// -----------------------------------------------------------------
	// TODO compute rotation matrix to convert from camera frame to gimbal frame
	// -----------------------------------------------------------------
	const float data[9] = { 0.0f, 0.0f, 1.0f,
							1.0f, 0.0f, 0.0f,
							0.0f, 1.0f, 0.0f};

	return data;
}

/** Compute a rotation matrix to convert from gimbal frame to the drone's body frame using the gimbal angles
 *  @param pitch angle of the gimble
 *  @param yaw angle of the gimble
 *  @return the computed rotation matrix
 */
matrix::Dcm<float> TargetDetector::compute_rotation_gimbal_to_drone(
	const float pitch,
	const float yaw)
{
	// -----------------------------------------------------------------
	// TODO compute rotation matrix to convert from gimbal frame to the
	//      drone's body frame using gimbal angles
	// -----------------------------------------------------------------
	//const float data[9] = { 1.0f, 0.0f, 0.0f,
							//0.0f, 1.0f, 0.0f,
							//0.0f, 0.0f, 1.0f};
	const matrix::Vector3f Euler = matrix::Euler<float>(0.0f,pitch,yaw);

	const float rot_matrix[9] = rotation_matrix(Euler);
	const float unrot_matrix[9] = inversed_rotation(rot_matrix.transpose());
	return unrot_matrix;
}

/** Compute the rotation matrix from image frame to local frame.
 *  @param image_rot rotation matrix from image frame to camera NED frame
 *  @param gimbal_rot rotation matrix from camera NED frame to drone's body
 *  @param att_vehicle rotation matrix
 *  @return the computed rotation matrix
 */
matrix::Dcm<float> TargetDetector::compute_rotation_matrix(
	const matrix::Dcm<float>& image_rot,
	const matrix::Dcm<float>& gimbal_rot,
	const matrix::Dcm<float>& att_vehicle)
{
	// -----------------------------------------------------------------
	// TODO compute the rotation matrix to convert from image frame to
	//      local frame
	// -----------------------------------------------------------------
	// M = (att_vehicle.inversed()*gimbal_rot*image_rot);
	return (att_vehicle.inversed()*gimbal_rot*image_rot);
}

/** Compute the target position in the local frame.
 *  @param total_rot the rotation matrix from image frame to local frame
 *  @param target_pos the position in image frame
 *  @param pos_vehicle the position of the drone
 *  @return target position in local frame
 */
matrix::Vector3f TargetDetector::compute_target_position_local_frame(
	const matrix::Dcm<float>& total_rot,
	const matrix::Vector3f& target_pos,
	const matrix::Vector3f& pos_vehicle)
{
	// -----------------------------------------------------------------
	// TODO convert target's position from image frame to local frame
	// -----------------------------------------------------------------
	matrix::Vector3f target = total_rot*target_pos + pos_vehicle;
	return target;
}

/** Convert covariance matrix from image frame to local frame
 *  @param var_if covariance matrix in image frame
 *  @param total_rot rotation matrix from image frame to local frame
 *  @return covariance matrix in local frame
 */
matrix::SquareMatrix<float, 3> TargetDetector::compute_covariance_local_frame(
	const matrix::SquareMatrix<float, 3>& var_if,
	const matrix::Dcm<float>& total_rot)
{
	// -----------------------------------------------------------------
	// TODO convert covariance from image frame to local frame
	// -----------------------------------------------------------------
	// Sadly, total.rot.I() != total_rot.transpose() for the desired precision
	// use the explicit matrix inverse to pass the unit test!
	return total_rot * var_if * total.I();
}

void TargetDetector::update_subscriptions()
{
	//--------------------------------------------------------------------------------------
	// TODO update subscriptions for vehicle_attitude as well as for vehicle_local_position
	//   and update the member variables _att_vehicle and _pos_vehicle
	// -------------------------------------------------------------------------------------
	bool updated_att
	vehicle_att_s vehicle_attitude_msg;

	orb_check(_vehicle_attitude_sub, &updated_att);

	while(!updated_att){
		orb_check(_vehicle_attitude_sub, &updated_att);
		}
	if (updated_att)
	{
		orb_copy(ORB_ID(vehicle_attitude), _vehicle_attitude_sub, &vehicle_attitude_msg);
		quaternion = vehicle_attitude_msg.q;
		_att_vehicle = rotation_matrix(quaternion.inversed()); 
	}

	bool updated_pos
	orb_check(_local_pos_sub, &updated_pos);
	if (updated_pos)
	 {
	 	vehicle_local_position_s local_pos;
	 	orb_copy(ORB_ID(vehicle_local_position), _local_pos_sub, &local_pos);
	 	pos_vehicle.x = local_pos.x;
	 	pos_vehicle.y = local_pos.y;
	 	pos_vehicle.z = -local_pos.z;
	 } 
}
