/**
 * @file target_following_params.c
 * TargetFollowing parameters
 *
 * Designed for the LIS-EPFL DroneCourse
 *
 * @author Basil Huber LIS <basil.huber@gmail.com>
 */


/**
* Position controller gain
*
* Gain of the proportional position controller
*
* @min 0.0
* @max 20.0
* @decimal 2
* @group Dronecourse
*/
PARAM_DEFINE_FLOAT(POS_GAIN, 0.3f);
PARAM_DEFINE_FLOAT(POS_ACCEPT_RAD, 2.0f);

// ----------------------------------------------------------------------------
// TODO: Add your parameter for the position controller acceptance radius
// ----------------------------------------------------------------------------

