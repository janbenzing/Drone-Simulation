#pragma once
// MESSAGE PLATFORM_LANDING PACKING

#define MAVLINK_MSG_ID_PLATFORM_LANDING 155

MAVPACKED(
typedef struct __mavlink_platform_landing_t {
 uint64_t time_usec; /*< Timestamp (micros since boot or Unix epoch)*/
 float vx; /*<  X Velocity*/
 float vy; /*<  Y Velocity*/
 float vz; /*<  Z Velocity*/
 uint8_t contact; /*<  1: true, 0: false*/
}) mavlink_platform_landing_t;

#define MAVLINK_MSG_ID_PLATFORM_LANDING_LEN 21
#define MAVLINK_MSG_ID_PLATFORM_LANDING_MIN_LEN 21
#define MAVLINK_MSG_ID_155_LEN 21
#define MAVLINK_MSG_ID_155_MIN_LEN 21

#define MAVLINK_MSG_ID_PLATFORM_LANDING_CRC 98
#define MAVLINK_MSG_ID_155_CRC 98



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_PLATFORM_LANDING { \
    155, \
    "PLATFORM_LANDING", \
    5, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_platform_landing_t, time_usec) }, \
         { "vx", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_platform_landing_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_platform_landing_t, vy) }, \
         { "vz", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_platform_landing_t, vz) }, \
         { "contact", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_platform_landing_t, contact) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_PLATFORM_LANDING { \
    "PLATFORM_LANDING", \
    5, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_platform_landing_t, time_usec) }, \
         { "vx", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_platform_landing_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_platform_landing_t, vy) }, \
         { "vz", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_platform_landing_t, vz) }, \
         { "contact", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_platform_landing_t, contact) }, \
         } \
}
#endif

/**
 * @brief Pack a platform_landing message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec Timestamp (micros since boot or Unix epoch)
 * @param vx  X Velocity
 * @param vy  Y Velocity
 * @param vz  Z Velocity
 * @param contact  1: true, 0: false
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_platform_landing_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_usec, float vx, float vy, float vz, uint8_t contact)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PLATFORM_LANDING_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, vx);
    _mav_put_float(buf, 12, vy);
    _mav_put_float(buf, 16, vz);
    _mav_put_uint8_t(buf, 20, contact);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PLATFORM_LANDING_LEN);
#else
    mavlink_platform_landing_t packet;
    packet.time_usec = time_usec;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;
    packet.contact = contact;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PLATFORM_LANDING_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PLATFORM_LANDING;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_PLATFORM_LANDING_MIN_LEN, MAVLINK_MSG_ID_PLATFORM_LANDING_LEN, MAVLINK_MSG_ID_PLATFORM_LANDING_CRC);
}

/**
 * @brief Pack a platform_landing message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec Timestamp (micros since boot or Unix epoch)
 * @param vx  X Velocity
 * @param vy  Y Velocity
 * @param vz  Z Velocity
 * @param contact  1: true, 0: false
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_platform_landing_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_usec,float vx,float vy,float vz,uint8_t contact)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PLATFORM_LANDING_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, vx);
    _mav_put_float(buf, 12, vy);
    _mav_put_float(buf, 16, vz);
    _mav_put_uint8_t(buf, 20, contact);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PLATFORM_LANDING_LEN);
#else
    mavlink_platform_landing_t packet;
    packet.time_usec = time_usec;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;
    packet.contact = contact;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PLATFORM_LANDING_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PLATFORM_LANDING;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_PLATFORM_LANDING_MIN_LEN, MAVLINK_MSG_ID_PLATFORM_LANDING_LEN, MAVLINK_MSG_ID_PLATFORM_LANDING_CRC);
}

/**
 * @brief Encode a platform_landing struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param platform_landing C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_platform_landing_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_platform_landing_t* platform_landing)
{
    return mavlink_msg_platform_landing_pack(system_id, component_id, msg, platform_landing->time_usec, platform_landing->vx, platform_landing->vy, platform_landing->vz, platform_landing->contact);
}

/**
 * @brief Encode a platform_landing struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param platform_landing C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_platform_landing_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_platform_landing_t* platform_landing)
{
    return mavlink_msg_platform_landing_pack_chan(system_id, component_id, chan, msg, platform_landing->time_usec, platform_landing->vx, platform_landing->vy, platform_landing->vz, platform_landing->contact);
}

/**
 * @brief Send a platform_landing message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec Timestamp (micros since boot or Unix epoch)
 * @param vx  X Velocity
 * @param vy  Y Velocity
 * @param vz  Z Velocity
 * @param contact  1: true, 0: false
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_platform_landing_send(mavlink_channel_t chan, uint64_t time_usec, float vx, float vy, float vz, uint8_t contact)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PLATFORM_LANDING_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, vx);
    _mav_put_float(buf, 12, vy);
    _mav_put_float(buf, 16, vz);
    _mav_put_uint8_t(buf, 20, contact);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PLATFORM_LANDING, buf, MAVLINK_MSG_ID_PLATFORM_LANDING_MIN_LEN, MAVLINK_MSG_ID_PLATFORM_LANDING_LEN, MAVLINK_MSG_ID_PLATFORM_LANDING_CRC);
#else
    mavlink_platform_landing_t packet;
    packet.time_usec = time_usec;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;
    packet.contact = contact;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PLATFORM_LANDING, (const char *)&packet, MAVLINK_MSG_ID_PLATFORM_LANDING_MIN_LEN, MAVLINK_MSG_ID_PLATFORM_LANDING_LEN, MAVLINK_MSG_ID_PLATFORM_LANDING_CRC);
#endif
}

/**
 * @brief Send a platform_landing message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_platform_landing_send_struct(mavlink_channel_t chan, const mavlink_platform_landing_t* platform_landing)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_platform_landing_send(chan, platform_landing->time_usec, platform_landing->vx, platform_landing->vy, platform_landing->vz, platform_landing->contact);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PLATFORM_LANDING, (const char *)platform_landing, MAVLINK_MSG_ID_PLATFORM_LANDING_MIN_LEN, MAVLINK_MSG_ID_PLATFORM_LANDING_LEN, MAVLINK_MSG_ID_PLATFORM_LANDING_CRC);
#endif
}

#if MAVLINK_MSG_ID_PLATFORM_LANDING_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_platform_landing_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, float vx, float vy, float vz, uint8_t contact)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, vx);
    _mav_put_float(buf, 12, vy);
    _mav_put_float(buf, 16, vz);
    _mav_put_uint8_t(buf, 20, contact);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PLATFORM_LANDING, buf, MAVLINK_MSG_ID_PLATFORM_LANDING_MIN_LEN, MAVLINK_MSG_ID_PLATFORM_LANDING_LEN, MAVLINK_MSG_ID_PLATFORM_LANDING_CRC);
#else
    mavlink_platform_landing_t *packet = (mavlink_platform_landing_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->vx = vx;
    packet->vy = vy;
    packet->vz = vz;
    packet->contact = contact;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PLATFORM_LANDING, (const char *)packet, MAVLINK_MSG_ID_PLATFORM_LANDING_MIN_LEN, MAVLINK_MSG_ID_PLATFORM_LANDING_LEN, MAVLINK_MSG_ID_PLATFORM_LANDING_CRC);
#endif
}
#endif

#endif

// MESSAGE PLATFORM_LANDING UNPACKING


/**
 * @brief Get field time_usec from platform_landing message
 *
 * @return Timestamp (micros since boot or Unix epoch)
 */
static inline uint64_t mavlink_msg_platform_landing_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field vx from platform_landing message
 *
 * @return  X Velocity
 */
static inline float mavlink_msg_platform_landing_get_vx(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field vy from platform_landing message
 *
 * @return  Y Velocity
 */
static inline float mavlink_msg_platform_landing_get_vy(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field vz from platform_landing message
 *
 * @return  Z Velocity
 */
static inline float mavlink_msg_platform_landing_get_vz(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field contact from platform_landing message
 *
 * @return  1: true, 0: false
 */
static inline uint8_t mavlink_msg_platform_landing_get_contact(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  20);
}

/**
 * @brief Decode a platform_landing message into a struct
 *
 * @param msg The message to decode
 * @param platform_landing C-struct to decode the message contents into
 */
static inline void mavlink_msg_platform_landing_decode(const mavlink_message_t* msg, mavlink_platform_landing_t* platform_landing)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    platform_landing->time_usec = mavlink_msg_platform_landing_get_time_usec(msg);
    platform_landing->vx = mavlink_msg_platform_landing_get_vx(msg);
    platform_landing->vy = mavlink_msg_platform_landing_get_vy(msg);
    platform_landing->vz = mavlink_msg_platform_landing_get_vz(msg);
    platform_landing->contact = mavlink_msg_platform_landing_get_contact(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_PLATFORM_LANDING_LEN? msg->len : MAVLINK_MSG_ID_PLATFORM_LANDING_LEN;
        memset(platform_landing, 0, MAVLINK_MSG_ID_PLATFORM_LANDING_LEN);
    memcpy(platform_landing, _MAV_PAYLOAD(msg), len);
#endif
}
