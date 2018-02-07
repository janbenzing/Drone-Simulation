#pragma once
// MESSAGE GIMBAL_COMMAND PACKING

#define MAVLINK_MSG_ID_GIMBAL_COMMAND 153

MAVPACKED(
typedef struct __mavlink_gimbal_command_t {
 uint64_t time_usec; /*< Timestamp (micros since boot or Unix epoch)*/
 float pitch; /*<  Pitch from body to camera frame*/
 float yaw; /*<  Yaw from body to camera frame*/
}) mavlink_gimbal_command_t;

#define MAVLINK_MSG_ID_GIMBAL_COMMAND_LEN 16
#define MAVLINK_MSG_ID_GIMBAL_COMMAND_MIN_LEN 16
#define MAVLINK_MSG_ID_153_LEN 16
#define MAVLINK_MSG_ID_153_MIN_LEN 16

#define MAVLINK_MSG_ID_GIMBAL_COMMAND_CRC 109
#define MAVLINK_MSG_ID_153_CRC 109



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_GIMBAL_COMMAND { \
    153, \
    "GIMBAL_COMMAND", \
    3, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_gimbal_command_t, time_usec) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_gimbal_command_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_gimbal_command_t, yaw) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_GIMBAL_COMMAND { \
    "GIMBAL_COMMAND", \
    3, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_gimbal_command_t, time_usec) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_gimbal_command_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_gimbal_command_t, yaw) }, \
         } \
}
#endif

/**
 * @brief Pack a gimbal_command message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec Timestamp (micros since boot or Unix epoch)
 * @param pitch  Pitch from body to camera frame
 * @param yaw  Yaw from body to camera frame
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gimbal_command_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_usec, float pitch, float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GIMBAL_COMMAND_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, pitch);
    _mav_put_float(buf, 12, yaw);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GIMBAL_COMMAND_LEN);
#else
    mavlink_gimbal_command_t packet;
    packet.time_usec = time_usec;
    packet.pitch = pitch;
    packet.yaw = yaw;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GIMBAL_COMMAND_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GIMBAL_COMMAND;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GIMBAL_COMMAND_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_COMMAND_LEN, MAVLINK_MSG_ID_GIMBAL_COMMAND_CRC);
}

/**
 * @brief Pack a gimbal_command message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec Timestamp (micros since boot or Unix epoch)
 * @param pitch  Pitch from body to camera frame
 * @param yaw  Yaw from body to camera frame
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gimbal_command_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_usec,float pitch,float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GIMBAL_COMMAND_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, pitch);
    _mav_put_float(buf, 12, yaw);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GIMBAL_COMMAND_LEN);
#else
    mavlink_gimbal_command_t packet;
    packet.time_usec = time_usec;
    packet.pitch = pitch;
    packet.yaw = yaw;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GIMBAL_COMMAND_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GIMBAL_COMMAND;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GIMBAL_COMMAND_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_COMMAND_LEN, MAVLINK_MSG_ID_GIMBAL_COMMAND_CRC);
}

/**
 * @brief Encode a gimbal_command struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gimbal_command C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gimbal_command_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gimbal_command_t* gimbal_command)
{
    return mavlink_msg_gimbal_command_pack(system_id, component_id, msg, gimbal_command->time_usec, gimbal_command->pitch, gimbal_command->yaw);
}

/**
 * @brief Encode a gimbal_command struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gimbal_command C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gimbal_command_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_gimbal_command_t* gimbal_command)
{
    return mavlink_msg_gimbal_command_pack_chan(system_id, component_id, chan, msg, gimbal_command->time_usec, gimbal_command->pitch, gimbal_command->yaw);
}

/**
 * @brief Send a gimbal_command message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec Timestamp (micros since boot or Unix epoch)
 * @param pitch  Pitch from body to camera frame
 * @param yaw  Yaw from body to camera frame
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gimbal_command_send(mavlink_channel_t chan, uint64_t time_usec, float pitch, float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GIMBAL_COMMAND_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, pitch);
    _mav_put_float(buf, 12, yaw);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_COMMAND, buf, MAVLINK_MSG_ID_GIMBAL_COMMAND_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_COMMAND_LEN, MAVLINK_MSG_ID_GIMBAL_COMMAND_CRC);
#else
    mavlink_gimbal_command_t packet;
    packet.time_usec = time_usec;
    packet.pitch = pitch;
    packet.yaw = yaw;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_COMMAND, (const char *)&packet, MAVLINK_MSG_ID_GIMBAL_COMMAND_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_COMMAND_LEN, MAVLINK_MSG_ID_GIMBAL_COMMAND_CRC);
#endif
}

/**
 * @brief Send a gimbal_command message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_gimbal_command_send_struct(mavlink_channel_t chan, const mavlink_gimbal_command_t* gimbal_command)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_gimbal_command_send(chan, gimbal_command->time_usec, gimbal_command->pitch, gimbal_command->yaw);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_COMMAND, (const char *)gimbal_command, MAVLINK_MSG_ID_GIMBAL_COMMAND_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_COMMAND_LEN, MAVLINK_MSG_ID_GIMBAL_COMMAND_CRC);
#endif
}

#if MAVLINK_MSG_ID_GIMBAL_COMMAND_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_gimbal_command_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, float pitch, float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, pitch);
    _mav_put_float(buf, 12, yaw);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_COMMAND, buf, MAVLINK_MSG_ID_GIMBAL_COMMAND_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_COMMAND_LEN, MAVLINK_MSG_ID_GIMBAL_COMMAND_CRC);
#else
    mavlink_gimbal_command_t *packet = (mavlink_gimbal_command_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->pitch = pitch;
    packet->yaw = yaw;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_COMMAND, (const char *)packet, MAVLINK_MSG_ID_GIMBAL_COMMAND_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_COMMAND_LEN, MAVLINK_MSG_ID_GIMBAL_COMMAND_CRC);
#endif
}
#endif

#endif

// MESSAGE GIMBAL_COMMAND UNPACKING


/**
 * @brief Get field time_usec from gimbal_command message
 *
 * @return Timestamp (micros since boot or Unix epoch)
 */
static inline uint64_t mavlink_msg_gimbal_command_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field pitch from gimbal_command message
 *
 * @return  Pitch from body to camera frame
 */
static inline float mavlink_msg_gimbal_command_get_pitch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field yaw from gimbal_command message
 *
 * @return  Yaw from body to camera frame
 */
static inline float mavlink_msg_gimbal_command_get_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Decode a gimbal_command message into a struct
 *
 * @param msg The message to decode
 * @param gimbal_command C-struct to decode the message contents into
 */
static inline void mavlink_msg_gimbal_command_decode(const mavlink_message_t* msg, mavlink_gimbal_command_t* gimbal_command)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    gimbal_command->time_usec = mavlink_msg_gimbal_command_get_time_usec(msg);
    gimbal_command->pitch = mavlink_msg_gimbal_command_get_pitch(msg);
    gimbal_command->yaw = mavlink_msg_gimbal_command_get_yaw(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_GIMBAL_COMMAND_LEN? msg->len : MAVLINK_MSG_ID_GIMBAL_COMMAND_LEN;
        memset(gimbal_command, 0, MAVLINK_MSG_ID_GIMBAL_COMMAND_LEN);
    memcpy(gimbal_command, _MAV_PAYLOAD(msg), len);
#endif
}
