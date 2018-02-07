#pragma once
// MESSAGE TARGET_POSITION_NED PACKING

#define MAVLINK_MSG_ID_TARGET_POSITION_NED 150

MAVPACKED(
typedef struct __mavlink_target_position_ned_t {
 float x; /*< X Position in NED frame in meters*/
 float y; /*< Y Position in NED frame in meters*/
 float z; /*< Z Position in NED frame in meters*/
 float var_x; /*< X Position variance in NED frame in meters^2*/
 float var_y; /*< Y Position variance in NED frame in meters^2*/
 float var_z; /*< Z Position variance in NED frame in meters^2*/
 uint8_t target_num; /*< ID of the target object.*/
}) mavlink_target_position_ned_t;

#define MAVLINK_MSG_ID_TARGET_POSITION_NED_LEN 25
#define MAVLINK_MSG_ID_TARGET_POSITION_NED_MIN_LEN 25
#define MAVLINK_MSG_ID_150_LEN 25
#define MAVLINK_MSG_ID_150_MIN_LEN 25

#define MAVLINK_MSG_ID_TARGET_POSITION_NED_CRC 206
#define MAVLINK_MSG_ID_150_CRC 206



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_TARGET_POSITION_NED { \
    150, \
    "TARGET_POSITION_NED", \
    7, \
    {  { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_target_position_ned_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_target_position_ned_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_target_position_ned_t, z) }, \
         { "var_x", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_target_position_ned_t, var_x) }, \
         { "var_y", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_target_position_ned_t, var_y) }, \
         { "var_z", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_target_position_ned_t, var_z) }, \
         { "target_num", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_target_position_ned_t, target_num) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_TARGET_POSITION_NED { \
    "TARGET_POSITION_NED", \
    7, \
    {  { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_target_position_ned_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_target_position_ned_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_target_position_ned_t, z) }, \
         { "var_x", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_target_position_ned_t, var_x) }, \
         { "var_y", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_target_position_ned_t, var_y) }, \
         { "var_z", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_target_position_ned_t, var_z) }, \
         { "target_num", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_target_position_ned_t, target_num) }, \
         } \
}
#endif

/**
 * @brief Pack a target_position_ned message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param x X Position in NED frame in meters
 * @param y Y Position in NED frame in meters
 * @param z Z Position in NED frame in meters
 * @param var_x X Position variance in NED frame in meters^2
 * @param var_y Y Position variance in NED frame in meters^2
 * @param var_z Z Position variance in NED frame in meters^2
 * @param target_num ID of the target object.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_target_position_ned_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float x, float y, float z, float var_x, float var_y, float var_z, uint8_t target_num)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TARGET_POSITION_NED_LEN];
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, z);
    _mav_put_float(buf, 12, var_x);
    _mav_put_float(buf, 16, var_y);
    _mav_put_float(buf, 20, var_z);
    _mav_put_uint8_t(buf, 24, target_num);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TARGET_POSITION_NED_LEN);
#else
    mavlink_target_position_ned_t packet;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.var_x = var_x;
    packet.var_y = var_y;
    packet.var_z = var_z;
    packet.target_num = target_num;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TARGET_POSITION_NED_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TARGET_POSITION_NED;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_TARGET_POSITION_NED_MIN_LEN, MAVLINK_MSG_ID_TARGET_POSITION_NED_LEN, MAVLINK_MSG_ID_TARGET_POSITION_NED_CRC);
}

/**
 * @brief Pack a target_position_ned message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param x X Position in NED frame in meters
 * @param y Y Position in NED frame in meters
 * @param z Z Position in NED frame in meters
 * @param var_x X Position variance in NED frame in meters^2
 * @param var_y Y Position variance in NED frame in meters^2
 * @param var_z Z Position variance in NED frame in meters^2
 * @param target_num ID of the target object.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_target_position_ned_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float x,float y,float z,float var_x,float var_y,float var_z,uint8_t target_num)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TARGET_POSITION_NED_LEN];
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, z);
    _mav_put_float(buf, 12, var_x);
    _mav_put_float(buf, 16, var_y);
    _mav_put_float(buf, 20, var_z);
    _mav_put_uint8_t(buf, 24, target_num);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TARGET_POSITION_NED_LEN);
#else
    mavlink_target_position_ned_t packet;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.var_x = var_x;
    packet.var_y = var_y;
    packet.var_z = var_z;
    packet.target_num = target_num;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TARGET_POSITION_NED_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TARGET_POSITION_NED;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_TARGET_POSITION_NED_MIN_LEN, MAVLINK_MSG_ID_TARGET_POSITION_NED_LEN, MAVLINK_MSG_ID_TARGET_POSITION_NED_CRC);
}

/**
 * @brief Encode a target_position_ned struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param target_position_ned C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_target_position_ned_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_target_position_ned_t* target_position_ned)
{
    return mavlink_msg_target_position_ned_pack(system_id, component_id, msg, target_position_ned->x, target_position_ned->y, target_position_ned->z, target_position_ned->var_x, target_position_ned->var_y, target_position_ned->var_z, target_position_ned->target_num);
}

/**
 * @brief Encode a target_position_ned struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_position_ned C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_target_position_ned_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_target_position_ned_t* target_position_ned)
{
    return mavlink_msg_target_position_ned_pack_chan(system_id, component_id, chan, msg, target_position_ned->x, target_position_ned->y, target_position_ned->z, target_position_ned->var_x, target_position_ned->var_y, target_position_ned->var_z, target_position_ned->target_num);
}

/**
 * @brief Send a target_position_ned message
 * @param chan MAVLink channel to send the message
 *
 * @param x X Position in NED frame in meters
 * @param y Y Position in NED frame in meters
 * @param z Z Position in NED frame in meters
 * @param var_x X Position variance in NED frame in meters^2
 * @param var_y Y Position variance in NED frame in meters^2
 * @param var_z Z Position variance in NED frame in meters^2
 * @param target_num ID of the target object.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_target_position_ned_send(mavlink_channel_t chan, float x, float y, float z, float var_x, float var_y, float var_z, uint8_t target_num)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TARGET_POSITION_NED_LEN];
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, z);
    _mav_put_float(buf, 12, var_x);
    _mav_put_float(buf, 16, var_y);
    _mav_put_float(buf, 20, var_z);
    _mav_put_uint8_t(buf, 24, target_num);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TARGET_POSITION_NED, buf, MAVLINK_MSG_ID_TARGET_POSITION_NED_MIN_LEN, MAVLINK_MSG_ID_TARGET_POSITION_NED_LEN, MAVLINK_MSG_ID_TARGET_POSITION_NED_CRC);
#else
    mavlink_target_position_ned_t packet;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.var_x = var_x;
    packet.var_y = var_y;
    packet.var_z = var_z;
    packet.target_num = target_num;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TARGET_POSITION_NED, (const char *)&packet, MAVLINK_MSG_ID_TARGET_POSITION_NED_MIN_LEN, MAVLINK_MSG_ID_TARGET_POSITION_NED_LEN, MAVLINK_MSG_ID_TARGET_POSITION_NED_CRC);
#endif
}

/**
 * @brief Send a target_position_ned message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_target_position_ned_send_struct(mavlink_channel_t chan, const mavlink_target_position_ned_t* target_position_ned)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_target_position_ned_send(chan, target_position_ned->x, target_position_ned->y, target_position_ned->z, target_position_ned->var_x, target_position_ned->var_y, target_position_ned->var_z, target_position_ned->target_num);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TARGET_POSITION_NED, (const char *)target_position_ned, MAVLINK_MSG_ID_TARGET_POSITION_NED_MIN_LEN, MAVLINK_MSG_ID_TARGET_POSITION_NED_LEN, MAVLINK_MSG_ID_TARGET_POSITION_NED_CRC);
#endif
}

#if MAVLINK_MSG_ID_TARGET_POSITION_NED_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_target_position_ned_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float x, float y, float z, float var_x, float var_y, float var_z, uint8_t target_num)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, z);
    _mav_put_float(buf, 12, var_x);
    _mav_put_float(buf, 16, var_y);
    _mav_put_float(buf, 20, var_z);
    _mav_put_uint8_t(buf, 24, target_num);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TARGET_POSITION_NED, buf, MAVLINK_MSG_ID_TARGET_POSITION_NED_MIN_LEN, MAVLINK_MSG_ID_TARGET_POSITION_NED_LEN, MAVLINK_MSG_ID_TARGET_POSITION_NED_CRC);
#else
    mavlink_target_position_ned_t *packet = (mavlink_target_position_ned_t *)msgbuf;
    packet->x = x;
    packet->y = y;
    packet->z = z;
    packet->var_x = var_x;
    packet->var_y = var_y;
    packet->var_z = var_z;
    packet->target_num = target_num;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TARGET_POSITION_NED, (const char *)packet, MAVLINK_MSG_ID_TARGET_POSITION_NED_MIN_LEN, MAVLINK_MSG_ID_TARGET_POSITION_NED_LEN, MAVLINK_MSG_ID_TARGET_POSITION_NED_CRC);
#endif
}
#endif

#endif

// MESSAGE TARGET_POSITION_NED UNPACKING


/**
 * @brief Get field x from target_position_ned message
 *
 * @return X Position in NED frame in meters
 */
static inline float mavlink_msg_target_position_ned_get_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field y from target_position_ned message
 *
 * @return Y Position in NED frame in meters
 */
static inline float mavlink_msg_target_position_ned_get_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field z from target_position_ned message
 *
 * @return Z Position in NED frame in meters
 */
static inline float mavlink_msg_target_position_ned_get_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field var_x from target_position_ned message
 *
 * @return X Position variance in NED frame in meters^2
 */
static inline float mavlink_msg_target_position_ned_get_var_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field var_y from target_position_ned message
 *
 * @return Y Position variance in NED frame in meters^2
 */
static inline float mavlink_msg_target_position_ned_get_var_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field var_z from target_position_ned message
 *
 * @return Z Position variance in NED frame in meters^2
 */
static inline float mavlink_msg_target_position_ned_get_var_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field target_num from target_position_ned message
 *
 * @return ID of the target object.
 */
static inline uint8_t mavlink_msg_target_position_ned_get_target_num(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  24);
}

/**
 * @brief Decode a target_position_ned message into a struct
 *
 * @param msg The message to decode
 * @param target_position_ned C-struct to decode the message contents into
 */
static inline void mavlink_msg_target_position_ned_decode(const mavlink_message_t* msg, mavlink_target_position_ned_t* target_position_ned)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    target_position_ned->x = mavlink_msg_target_position_ned_get_x(msg);
    target_position_ned->y = mavlink_msg_target_position_ned_get_y(msg);
    target_position_ned->z = mavlink_msg_target_position_ned_get_z(msg);
    target_position_ned->var_x = mavlink_msg_target_position_ned_get_var_x(msg);
    target_position_ned->var_y = mavlink_msg_target_position_ned_get_var_y(msg);
    target_position_ned->var_z = mavlink_msg_target_position_ned_get_var_z(msg);
    target_position_ned->target_num = mavlink_msg_target_position_ned_get_target_num(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_TARGET_POSITION_NED_LEN? msg->len : MAVLINK_MSG_ID_TARGET_POSITION_NED_LEN;
        memset(target_position_ned, 0, MAVLINK_MSG_ID_TARGET_POSITION_NED_LEN);
    memcpy(target_position_ned, _MAV_PAYLOAD(msg), len);
#endif
}
