/**
 * @file params.c
 * TargetTracking parameters
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
PARAM_DEFINE_FLOAT(XNOISE, 0.1f);

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
PARAM_DEFINE_FLOAT(YNOISE, 0.1f);

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
PARAM_DEFINE_FLOAT(ZNOISE, 0.1f);

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
PARAM_DEFINE_FLOAT(VXNOISE, 0.1f);

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
PARAM_DEFINE_FLOAT(VYNOISE, 0.1f);

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
PARAM_DEFINE_FLOAT(VZNOISE, 0.1f);
