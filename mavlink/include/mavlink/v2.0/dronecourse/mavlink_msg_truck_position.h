#pragma once
// MESSAGE TRUCK_POSITION PACKING

#define MAVLINK_MSG_ID_TRUCK_POSITION 157

MAVPACKED(
typedef struct __mavlink_truck_position_t {
 uint64_t time_usec; /*< Timestamp (micros since boot or Unix epoch)*/
 float x; /*<  X Position*/
 float y; /*<  Y Position*/
 float z; /*<  Z Position*/
 float vx; /*<  X Velocity*/
 float vy; /*<  Y Velocity*/
 float vz; /*<  Z Velocity*/
 float heading; /*<  Heading*/
}) mavlink_truck_position_t;

#define MAVLINK_MSG_ID_TRUCK_POSITION_LEN 36
#define MAVLINK_MSG_ID_TRUCK_POSITION_MIN_LEN 36
#define MAVLINK_MSG_ID_157_LEN 36
#define MAVLINK_MSG_ID_157_MIN_LEN 36

#define MAVLINK_MSG_ID_TRUCK_POSITION_CRC 136
#define MAVLINK_MSG_ID_157_CRC 136



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_TRUCK_POSITION { \
    157, \
    "TRUCK_POSITION", \
    8, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_truck_position_t, time_usec) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_truck_position_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_truck_position_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_truck_position_t, z) }, \
         { "vx", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_truck_position_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_truck_position_t, vy) }, \
         { "vz", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_truck_position_t, vz) }, \
         { "heading", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_truck_position_t, heading) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_TRUCK_POSITION { \
    "TRUCK_POSITION", \
    8, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_truck_position_t, time_usec) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_truck_position_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_truck_position_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_truck_position_t, z) }, \
         { "vx", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_truck_position_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_truck_position_t, vy) }, \
         { "vz", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_truck_position_t, vz) }, \
         { "heading", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_truck_position_t, heading) }, \
         } \
}
#endif

/**
 * @brief Pack a truck_position message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec Timestamp (micros since boot or Unix epoch)
 * @param x  X Position
 * @param y  Y Position
 * @param z  Z Position
 * @param vx  X Velocity
 * @param vy  Y Velocity
 * @param vz  Z Velocity
 * @param heading  Heading
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_truck_position_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_usec, float x, float y, float z, float vx, float vy, float vz, float heading)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TRUCK_POSITION_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, x);
    _mav_put_float(buf, 12, y);
    _mav_put_float(buf, 16, z);
    _mav_put_float(buf, 20, vx);
    _mav_put_float(buf, 24, vy);
    _mav_put_float(buf, 28, vz);
    _mav_put_float(buf, 32, heading);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TRUCK_POSITION_LEN);
#else
    mavlink_truck_position_t packet;
    packet.time_usec = time_usec;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;
    packet.heading = heading;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TRUCK_POSITION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TRUCK_POSITION;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_TRUCK_POSITION_MIN_LEN, MAVLINK_MSG_ID_TRUCK_POSITION_LEN, MAVLINK_MSG_ID_TRUCK_POSITION_CRC);
}

/**
 * @brief Pack a truck_position message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec Timestamp (micros since boot or Unix epoch)
 * @param x  X Position
 * @param y  Y Position
 * @param z  Z Position
 * @param vx  X Velocity
 * @param vy  Y Velocity
 * @param vz  Z Velocity
 * @param heading  Heading
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_truck_position_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_usec,float x,float y,float z,float vx,float vy,float vz,float heading)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TRUCK_POSITION_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, x);
    _mav_put_float(buf, 12, y);
    _mav_put_float(buf, 16, z);
    _mav_put_float(buf, 20, vx);
    _mav_put_float(buf, 24, vy);
    _mav_put_float(buf, 28, vz);
    _mav_put_float(buf, 32, heading);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TRUCK_POSITION_LEN);
#else
    mavlink_truck_position_t packet;
    packet.time_usec = time_usec;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;
    packet.heading = heading;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TRUCK_POSITION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TRUCK_POSITION;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_TRUCK_POSITION_MIN_LEN, MAVLINK_MSG_ID_TRUCK_POSITION_LEN, MAVLINK_MSG_ID_TRUCK_POSITION_CRC);
}

/**
 * @brief Encode a truck_position struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param truck_position C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_truck_position_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_truck_position_t* truck_position)
{
    return mavlink_msg_truck_position_pack(system_id, component_id, msg, truck_position->time_usec, truck_position->x, truck_position->y, truck_position->z, truck_position->vx, truck_position->vy, truck_position->vz, truck_position->heading);
}

/**
 * @brief Encode a truck_position struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param truck_position C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_truck_position_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_truck_position_t* truck_position)
{
    return mavlink_msg_truck_position_pack_chan(system_id, component_id, chan, msg, truck_position->time_usec, truck_position->x, truck_position->y, truck_position->z, truck_position->vx, truck_position->vy, truck_position->vz, truck_position->heading);
}

/**
 * @brief Send a truck_position message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec Timestamp (micros since boot or Unix epoch)
 * @param x  X Position
 * @param y  Y Position
 * @param z  Z Position
 * @param vx  X Velocity
 * @param vy  Y Velocity
 * @param vz  Z Velocity
 * @param heading  Heading
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_truck_position_send(mavlink_channel_t chan, uint64_t time_usec, float x, float y, float z, float vx, float vy, float vz, float heading)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TRUCK_POSITION_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, x);
    _mav_put_float(buf, 12, y);
    _mav_put_float(buf, 16, z);
    _mav_put_float(buf, 20, vx);
    _mav_put_float(buf, 24, vy);
    _mav_put_float(buf, 28, vz);
    _mav_put_float(buf, 32, heading);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TRUCK_POSITION, buf, MAVLINK_MSG_ID_TRUCK_POSITION_MIN_LEN, MAVLINK_MSG_ID_TRUCK_POSITION_LEN, MAVLINK_MSG_ID_TRUCK_POSITION_CRC);
#else
    mavlink_truck_position_t packet;
    packet.time_usec = time_usec;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;
    packet.heading = heading;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TRUCK_POSITION, (const char *)&packet, MAVLINK_MSG_ID_TRUCK_POSITION_MIN_LEN, MAVLINK_MSG_ID_TRUCK_POSITION_LEN, MAVLINK_MSG_ID_TRUCK_POSITION_CRC);
#endif
}

/**
 * @brief Send a truck_position message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_truck_position_send_struct(mavlink_channel_t chan, const mavlink_truck_position_t* truck_position)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_truck_position_send(chan, truck_position->time_usec, truck_position->x, truck_position->y, truck_position->z, truck_position->vx, truck_position->vy, truck_position->vz, truck_position->heading);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TRUCK_POSITION, (const char *)truck_position, MAVLINK_MSG_ID_TRUCK_POSITION_MIN_LEN, MAVLINK_MSG_ID_TRUCK_POSITION_LEN, MAVLINK_MSG_ID_TRUCK_POSITION_CRC);
#endif
}

#if MAVLINK_MSG_ID_TRUCK_POSITION_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_truck_position_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, float x, float y, float z, float vx, float vy, float vz, float heading)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, x);
    _mav_put_float(buf, 12, y);
    _mav_put_float(buf, 16, z);
    _mav_put_float(buf, 20, vx);
    _mav_put_float(buf, 24, vy);
    _mav_put_float(buf, 28, vz);
    _mav_put_float(buf, 32, heading);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TRUCK_POSITION, buf, MAVLINK_MSG_ID_TRUCK_POSITION_MIN_LEN, MAVLINK_MSG_ID_TRUCK_POSITION_LEN, MAVLINK_MSG_ID_TRUCK_POSITION_CRC);
#else
    mavlink_truck_position_t *packet = (mavlink_truck_position_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->x = x;
    packet->y = y;
    packet->z = z;
    packet->vx = vx;
    packet->vy = vy;
    packet->vz = vz;
    packet->heading = heading;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TRUCK_POSITION, (const char *)packet, MAVLINK_MSG_ID_TRUCK_POSITION_MIN_LEN, MAVLINK_MSG_ID_TRUCK_POSITION_LEN, MAVLINK_MSG_ID_TRUCK_POSITION_CRC);
#endif
}
#endif

#endif

// MESSAGE TRUCK_POSITION UNPACKING


/**
 * @brief Get field time_usec from truck_position message
 *
 * @return Timestamp (micros since boot or Unix epoch)
 */
static inline uint64_t mavlink_msg_truck_position_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field x from truck_position message
 *
 * @return  X Position
 */
static inline float mavlink_msg_truck_position_get_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field y from truck_position message
 *
 * @return  Y Position
 */
static inline float mavlink_msg_truck_position_get_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field z from truck_position message
 *
 * @return  Z Position
 */
static inline float mavlink_msg_truck_position_get_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field vx from truck_position message
 *
 * @return  X Velocity
 */
static inline float mavlink_msg_truck_position_get_vx(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field vy from truck_position message
 *
 * @return  Y Velocity
 */
static inline float mavlink_msg_truck_position_get_vy(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field vz from truck_position message
 *
 * @return  Z Velocity
 */
static inline float mavlink_msg_truck_position_get_vz(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field heading from truck_position message
 *
 * @return  Heading
 */
static inline float mavlink_msg_truck_position_get_heading(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Decode a truck_position message into a struct
 *
 * @param msg The message to decode
 * @param truck_position C-struct to decode the message contents into
 */
static inline void mavlink_msg_truck_position_decode(const mavlink_message_t* msg, mavlink_truck_position_t* truck_position)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    truck_position->time_usec = mavlink_msg_truck_position_get_time_usec(msg);
    truck_position->x = mavlink_msg_truck_position_get_x(msg);
    truck_position->y = mavlink_msg_truck_position_get_y(msg);
    truck_position->z = mavlink_msg_truck_position_get_z(msg);
    truck_position->vx = mavlink_msg_truck_position_get_vx(msg);
    truck_position->vy = mavlink_msg_truck_position_get_vy(msg);
    truck_position->vz = mavlink_msg_truck_position_get_vz(msg);
    truck_position->heading = mavlink_msg_truck_position_get_heading(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_TRUCK_POSITION_LEN? msg->len : MAVLINK_MSG_ID_TRUCK_POSITION_LEN;
        memset(truck_position, 0, MAVLINK_MSG_ID_TRUCK_POSITION_LEN);
    memcpy(truck_position, _MAV_PAYLOAD(msg), len);
#endif
}
