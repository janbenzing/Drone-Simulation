#pragma once
// MESSAGE PLATFORM_POSITION PACKING

#define MAVLINK_MSG_ID_PLATFORM_POSITION 156

MAVPACKED(
typedef struct __mavlink_platform_position_t {
 uint64_t time_usec; /*< Timestamp (micros since boot or Unix epoch)*/
 float x; /*<  X Position*/
 float y; /*<  Y Position*/
 float z; /*<  Z Position*/
}) mavlink_platform_position_t;

#define MAVLINK_MSG_ID_PLATFORM_POSITION_LEN 20
#define MAVLINK_MSG_ID_PLATFORM_POSITION_MIN_LEN 20
#define MAVLINK_MSG_ID_156_LEN 20
#define MAVLINK_MSG_ID_156_MIN_LEN 20

#define MAVLINK_MSG_ID_PLATFORM_POSITION_CRC 215
#define MAVLINK_MSG_ID_156_CRC 215



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_PLATFORM_POSITION { \
    156, \
    "PLATFORM_POSITION", \
    4, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_platform_position_t, time_usec) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_platform_position_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_platform_position_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_platform_position_t, z) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_PLATFORM_POSITION { \
    "PLATFORM_POSITION", \
    4, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_platform_position_t, time_usec) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_platform_position_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_platform_position_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_platform_position_t, z) }, \
         } \
}
#endif

/**
 * @brief Pack a platform_position message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec Timestamp (micros since boot or Unix epoch)
 * @param x  X Position
 * @param y  Y Position
 * @param z  Z Position
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_platform_position_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_usec, float x, float y, float z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PLATFORM_POSITION_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, x);
    _mav_put_float(buf, 12, y);
    _mav_put_float(buf, 16, z);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PLATFORM_POSITION_LEN);
#else
    mavlink_platform_position_t packet;
    packet.time_usec = time_usec;
    packet.x = x;
    packet.y = y;
    packet.z = z;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PLATFORM_POSITION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PLATFORM_POSITION;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_PLATFORM_POSITION_MIN_LEN, MAVLINK_MSG_ID_PLATFORM_POSITION_LEN, MAVLINK_MSG_ID_PLATFORM_POSITION_CRC);
}

/**
 * @brief Pack a platform_position message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec Timestamp (micros since boot or Unix epoch)
 * @param x  X Position
 * @param y  Y Position
 * @param z  Z Position
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_platform_position_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_usec,float x,float y,float z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PLATFORM_POSITION_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, x);
    _mav_put_float(buf, 12, y);
    _mav_put_float(buf, 16, z);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PLATFORM_POSITION_LEN);
#else
    mavlink_platform_position_t packet;
    packet.time_usec = time_usec;
    packet.x = x;
    packet.y = y;
    packet.z = z;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PLATFORM_POSITION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PLATFORM_POSITION;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_PLATFORM_POSITION_MIN_LEN, MAVLINK_MSG_ID_PLATFORM_POSITION_LEN, MAVLINK_MSG_ID_PLATFORM_POSITION_CRC);
}

/**
 * @brief Encode a platform_position struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param platform_position C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_platform_position_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_platform_position_t* platform_position)
{
    return mavlink_msg_platform_position_pack(system_id, component_id, msg, platform_position->time_usec, platform_position->x, platform_position->y, platform_position->z);
}

/**
 * @brief Encode a platform_position struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param platform_position C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_platform_position_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_platform_position_t* platform_position)
{
    return mavlink_msg_platform_position_pack_chan(system_id, component_id, chan, msg, platform_position->time_usec, platform_position->x, platform_position->y, platform_position->z);
}

/**
 * @brief Send a platform_position message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec Timestamp (micros since boot or Unix epoch)
 * @param x  X Position
 * @param y  Y Position
 * @param z  Z Position
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_platform_position_send(mavlink_channel_t chan, uint64_t time_usec, float x, float y, float z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PLATFORM_POSITION_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, x);
    _mav_put_float(buf, 12, y);
    _mav_put_float(buf, 16, z);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PLATFORM_POSITION, buf, MAVLINK_MSG_ID_PLATFORM_POSITION_MIN_LEN, MAVLINK_MSG_ID_PLATFORM_POSITION_LEN, MAVLINK_MSG_ID_PLATFORM_POSITION_CRC);
#else
    mavlink_platform_position_t packet;
    packet.time_usec = time_usec;
    packet.x = x;
    packet.y = y;
    packet.z = z;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PLATFORM_POSITION, (const char *)&packet, MAVLINK_MSG_ID_PLATFORM_POSITION_MIN_LEN, MAVLINK_MSG_ID_PLATFORM_POSITION_LEN, MAVLINK_MSG_ID_PLATFORM_POSITION_CRC);
#endif
}

/**
 * @brief Send a platform_position message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_platform_position_send_struct(mavlink_channel_t chan, const mavlink_platform_position_t* platform_position)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_platform_position_send(chan, platform_position->time_usec, platform_position->x, platform_position->y, platform_position->z);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PLATFORM_POSITION, (const char *)platform_position, MAVLINK_MSG_ID_PLATFORM_POSITION_MIN_LEN, MAVLINK_MSG_ID_PLATFORM_POSITION_LEN, MAVLINK_MSG_ID_PLATFORM_POSITION_CRC);
#endif
}

#if MAVLINK_MSG_ID_PLATFORM_POSITION_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_platform_position_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, float x, float y, float z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, x);
    _mav_put_float(buf, 12, y);
    _mav_put_float(buf, 16, z);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PLATFORM_POSITION, buf, MAVLINK_MSG_ID_PLATFORM_POSITION_MIN_LEN, MAVLINK_MSG_ID_PLATFORM_POSITION_LEN, MAVLINK_MSG_ID_PLATFORM_POSITION_CRC);
#else
    mavlink_platform_position_t *packet = (mavlink_platform_position_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->x = x;
    packet->y = y;
    packet->z = z;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PLATFORM_POSITION, (const char *)packet, MAVLINK_MSG_ID_PLATFORM_POSITION_MIN_LEN, MAVLINK_MSG_ID_PLATFORM_POSITION_LEN, MAVLINK_MSG_ID_PLATFORM_POSITION_CRC);
#endif
}
#endif

#endif

// MESSAGE PLATFORM_POSITION UNPACKING


/**
 * @brief Get field time_usec from platform_position message
 *
 * @return Timestamp (micros since boot or Unix epoch)
 */
static inline uint64_t mavlink_msg_platform_position_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field x from platform_position message
 *
 * @return  X Position
 */
static inline float mavlink_msg_platform_position_get_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field y from platform_position message
 *
 * @return  Y Position
 */
static inline float mavlink_msg_platform_position_get_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field z from platform_position message
 *
 * @return  Z Position
 */
static inline float mavlink_msg_platform_position_get_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Decode a platform_position message into a struct
 *
 * @param msg The message to decode
 * @param platform_position C-struct to decode the message contents into
 */
static inline void mavlink_msg_platform_position_decode(const mavlink_message_t* msg, mavlink_platform_position_t* platform_position)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    platform_position->time_usec = mavlink_msg_platform_position_get_time_usec(msg);
    platform_position->x = mavlink_msg_platform_position_get_x(msg);
    platform_position->y = mavlink_msg_platform_position_get_y(msg);
    platform_position->z = mavlink_msg_platform_position_get_z(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_PLATFORM_POSITION_LEN? msg->len : MAVLINK_MSG_ID_PLATFORM_POSITION_LEN;
        memset(platform_position, 0, MAVLINK_MSG_ID_PLATFORM_POSITION_LEN);
    memcpy(platform_position, _MAV_PAYLOAD(msg), len);
#endif
}
