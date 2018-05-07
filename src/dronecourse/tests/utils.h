/**
 * Utilities for unit tests.
 * @file utils.h
 * @author Arthur Gay <arthur.gay@epfl.ch>
 */

#pragma once

/** Compare two matrix __a, __b of same dimension __m, __n with
 * precision __fp
 */
#define compare_matrix(__msg, __a, __b, __m, __n, __fp) do { \
	for(int i = 0; i < __m; i++){ \
		for(int j = 0; j < __n; j++){\
			char buffer[255]; \
			snprintf(buffer, 255, __msg "(%d,%d)", i, j); \
			ut_compare_float(buffer, __a(i, j), __b(i, j), __fp); \
		} \
	} \
} while(0)

/** Compare two vectors __a, __b of same dimension __n with precision
 * __fp
 */
#define compare_vectors(__msg, __a, __b, __n, __fp) do { \
	for(int i = 0; i < __n; i++){ \
		char buffer[255]; \
		snprintf(buffer, 255, __msg "(%d)", i); \
		ut_compare_float(buffer, __a(i), __b(i), __fp); \
	} \
} while(0)


/** Compare two matrix::Quaternion __a and __b with precision __fp */
#define compare_quaternion(__msg, __a, __b, __fp) do { \
	ut_compare_float(__msg "(0)", __a(0), __b(0), __fp); \
	ut_compare_float(__msg "(1)", __a(1), __b(1), __fp); \
	ut_compare_float(__msg "(2)", __a(2), __b(2), __fp); \
	ut_compare_float(__msg "(3)", __a(3), __b(3), __fp); \
} while(0)

/** Compare two matrix::Vector3f __a and __b with precision __fp **/
#define compare_vector3f(__msg, __a, __b, __fp) do { \
	ut_compare_float(__msg "(0)", __a(0), __b(0), __fp); \
	ut_compare_float(__msg "(1)", __a(1), __b(1), __fp); \
	ut_compare_float(__msg "(2)", __a(2), __b(2), __fp); \
} while(0)

/** Compare two matrix::Vector2f __a and __b with precision __fp **/
#define compare_vector2f(__msg, __a, __b, __fp) do { \
	ut_compare_float(__msg "(0)", __a(0), __b(0), __fp); \
	ut_compare_float(__msg "(1)", __a(1), __b(1), __fp); \
} while(0)

/** Send a ORB message vehicle_local_position */
#define set_local_position_vector3f(__v) do { \
	vehicle_local_position_s __position_msg; \
	__position_msg.x = __v(0); \
	__position_msg.y = __v(1); \
	__position_msg.z = __v(2); \
	orb_advert_t __advertiser = orb_advertise(ORB_ID(vehicle_local_position), &__position_msg); \
	orb_unadvertise(__advertiser); \
} while(0)
