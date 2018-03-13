/** @file
 *    @brief MAVLink comm protocol testsuite generated from dronecourse.xml
 *    @see http://qgroundcontrol.org/mavlink/
 */
#pragma once
#ifndef DRONECOURSE_TESTSUITE_H
#define DRONECOURSE_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL
static void mavlink_test_common(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_dronecourse(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_common(system_id, component_id, last_msg);
    mavlink_test_dronecourse(system_id, component_id, last_msg);
}
#endif

#include "../common/testsuite.h"


static void mavlink_test_target_position_ned(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_TARGET_POSITION_NED >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_target_position_ned_t packet_in = {
        17.0,45.0,73.0,101.0,129.0,157.0,77
    };
    mavlink_target_position_ned_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.x = packet_in.x;
        packet1.y = packet_in.y;
        packet1.z = packet_in.z;
        packet1.var_x = packet_in.var_x;
        packet1.var_y = packet_in.var_y;
        packet1.var_z = packet_in.var_z;
        packet1.target_num = packet_in.target_num;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_TARGET_POSITION_NED_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_TARGET_POSITION_NED_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_target_position_ned_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_target_position_ned_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_target_position_ned_pack(system_id, component_id, &msg , packet1.x , packet1.y , packet1.z , packet1.var_x , packet1.var_y , packet1.var_z , packet1.target_num );
    mavlink_msg_target_position_ned_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_target_position_ned_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.x , packet1.y , packet1.z , packet1.var_x , packet1.var_y , packet1.var_z , packet1.target_num );
    mavlink_msg_target_position_ned_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_target_position_ned_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_target_position_ned_send(MAVLINK_COMM_1 , packet1.x , packet1.y , packet1.z , packet1.var_x , packet1.var_y , packet1.var_z , packet1.target_num );
    mavlink_msg_target_position_ned_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_target_position_ned_filtered(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_TARGET_POSITION_NED_FILTERED >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_target_position_ned_filtered_t packet_in = {
        17.0,45.0,73.0,101.0,129.0,157.0,185.0,213.0,241.0,269.0,297.0,325.0,149
    };
    mavlink_target_position_ned_filtered_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.x = packet_in.x;
        packet1.y = packet_in.y;
        packet1.z = packet_in.z;
        packet1.vx = packet_in.vx;
        packet1.vy = packet_in.vy;
        packet1.vz = packet_in.vz;
        packet1.var_x = packet_in.var_x;
        packet1.var_y = packet_in.var_y;
        packet1.var_z = packet_in.var_z;
        packet1.var_vx = packet_in.var_vx;
        packet1.var_vy = packet_in.var_vy;
        packet1.var_vz = packet_in.var_vz;
        packet1.target_num = packet_in.target_num;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_TARGET_POSITION_NED_FILTERED_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_TARGET_POSITION_NED_FILTERED_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_target_position_ned_filtered_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_target_position_ned_filtered_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_target_position_ned_filtered_pack(system_id, component_id, &msg , packet1.x , packet1.y , packet1.z , packet1.vx , packet1.vy , packet1.vz , packet1.var_x , packet1.var_y , packet1.var_z , packet1.var_vx , packet1.var_vy , packet1.var_vz , packet1.target_num );
    mavlink_msg_target_position_ned_filtered_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_target_position_ned_filtered_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.x , packet1.y , packet1.z , packet1.vx , packet1.vy , packet1.vz , packet1.var_x , packet1.var_y , packet1.var_z , packet1.var_vx , packet1.var_vy , packet1.var_vz , packet1.target_num );
    mavlink_msg_target_position_ned_filtered_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_target_position_ned_filtered_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_target_position_ned_filtered_send(MAVLINK_COMM_1 , packet1.x , packet1.y , packet1.z , packet1.vx , packet1.vy , packet1.vz , packet1.var_x , packet1.var_y , packet1.var_z , packet1.var_vx , packet1.var_vy , packet1.var_vz , packet1.target_num );
    mavlink_msg_target_position_ned_filtered_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_target_position_image(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_TARGET_POSITION_IMAGE >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_target_position_image_t packet_in = {
        93372036854775807ULL,963497880,963498088,129.0,157.0,185.0,213.0,241.0,269.0,125
    };
    mavlink_target_position_image_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_usec = packet_in.time_usec;
        packet1.u = packet_in.u;
        packet1.v = packet_in.v;
        packet1.dist = packet_in.dist;
        packet1.pitch = packet_in.pitch;
        packet1.yaw = packet_in.yaw;
        packet1.var_u = packet_in.var_u;
        packet1.var_v = packet_in.var_v;
        packet1.var_dist = packet_in.var_dist;
        packet1.target_num = packet_in.target_num;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_TARGET_POSITION_IMAGE_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_TARGET_POSITION_IMAGE_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_target_position_image_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_target_position_image_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_target_position_image_pack(system_id, component_id, &msg , packet1.time_usec , packet1.u , packet1.v , packet1.dist , packet1.pitch , packet1.yaw , packet1.var_u , packet1.var_v , packet1.var_dist , packet1.target_num );
    mavlink_msg_target_position_image_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_target_position_image_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_usec , packet1.u , packet1.v , packet1.dist , packet1.pitch , packet1.yaw , packet1.var_u , packet1.var_v , packet1.var_dist , packet1.target_num );
    mavlink_msg_target_position_image_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_target_position_image_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_target_position_image_send(MAVLINK_COMM_1 , packet1.time_usec , packet1.u , packet1.v , packet1.dist , packet1.pitch , packet1.yaw , packet1.var_u , packet1.var_v , packet1.var_dist , packet1.target_num );
    mavlink_msg_target_position_image_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_gimbal_command(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_GIMBAL_COMMAND >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_gimbal_command_t packet_in = {
        93372036854775807ULL,73.0,101.0
    };
    mavlink_gimbal_command_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_usec = packet_in.time_usec;
        packet1.pitch = packet_in.pitch;
        packet1.yaw = packet_in.yaw;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_GIMBAL_COMMAND_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_GIMBAL_COMMAND_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gimbal_command_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_gimbal_command_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gimbal_command_pack(system_id, component_id, &msg , packet1.time_usec , packet1.pitch , packet1.yaw );
    mavlink_msg_gimbal_command_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gimbal_command_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_usec , packet1.pitch , packet1.yaw );
    mavlink_msg_gimbal_command_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_gimbal_command_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gimbal_command_send(MAVLINK_COMM_1 , packet1.time_usec , packet1.pitch , packet1.yaw );
    mavlink_msg_gimbal_command_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_platform_landing(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_PLATFORM_LANDING >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_platform_landing_t packet_in = {
        93372036854775807ULL,73.0,101.0,129.0,65
    };
    mavlink_platform_landing_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_usec = packet_in.time_usec;
        packet1.vx = packet_in.vx;
        packet1.vy = packet_in.vy;
        packet1.vz = packet_in.vz;
        packet1.contact = packet_in.contact;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_PLATFORM_LANDING_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_PLATFORM_LANDING_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_platform_landing_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_platform_landing_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_platform_landing_pack(system_id, component_id, &msg , packet1.time_usec , packet1.vx , packet1.vy , packet1.vz , packet1.contact );
    mavlink_msg_platform_landing_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_platform_landing_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_usec , packet1.vx , packet1.vy , packet1.vz , packet1.contact );
    mavlink_msg_platform_landing_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_platform_landing_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_platform_landing_send(MAVLINK_COMM_1 , packet1.time_usec , packet1.vx , packet1.vy , packet1.vz , packet1.contact );
    mavlink_msg_platform_landing_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_platform_position(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_PLATFORM_POSITION >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_platform_position_t packet_in = {
        93372036854775807ULL,73.0,101.0,129.0
    };
    mavlink_platform_position_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_usec = packet_in.time_usec;
        packet1.x = packet_in.x;
        packet1.y = packet_in.y;
        packet1.z = packet_in.z;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_PLATFORM_POSITION_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_PLATFORM_POSITION_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_platform_position_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_platform_position_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_platform_position_pack(system_id, component_id, &msg , packet1.time_usec , packet1.x , packet1.y , packet1.z );
    mavlink_msg_platform_position_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_platform_position_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_usec , packet1.x , packet1.y , packet1.z );
    mavlink_msg_platform_position_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_platform_position_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_platform_position_send(MAVLINK_COMM_1 , packet1.time_usec , packet1.x , packet1.y , packet1.z );
    mavlink_msg_platform_position_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_truck_position(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_TRUCK_POSITION >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_truck_position_t packet_in = {
        93372036854775807ULL,73.0,101.0,129.0,157.0,185.0,213.0,241.0
    };
    mavlink_truck_position_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_usec = packet_in.time_usec;
        packet1.x = packet_in.x;
        packet1.y = packet_in.y;
        packet1.z = packet_in.z;
        packet1.vx = packet_in.vx;
        packet1.vy = packet_in.vy;
        packet1.vz = packet_in.vz;
        packet1.heading = packet_in.heading;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_TRUCK_POSITION_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_TRUCK_POSITION_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_truck_position_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_truck_position_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_truck_position_pack(system_id, component_id, &msg , packet1.time_usec , packet1.x , packet1.y , packet1.z , packet1.vx , packet1.vy , packet1.vz , packet1.heading );
    mavlink_msg_truck_position_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_truck_position_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_usec , packet1.x , packet1.y , packet1.z , packet1.vx , packet1.vy , packet1.vz , packet1.heading );
    mavlink_msg_truck_position_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_truck_position_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_truck_position_send(MAVLINK_COMM_1 , packet1.time_usec , packet1.x , packet1.y , packet1.z , packet1.vx , packet1.vy , packet1.vz , packet1.heading );
    mavlink_msg_truck_position_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_dronecourse(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_target_position_ned(system_id, component_id, last_msg);
    mavlink_test_target_position_ned_filtered(system_id, component_id, last_msg);
    mavlink_test_target_position_image(system_id, component_id, last_msg);
    mavlink_test_gimbal_command(system_id, component_id, last_msg);
    mavlink_test_platform_landing(system_id, component_id, last_msg);
    mavlink_test_platform_position(system_id, component_id, last_msg);
    mavlink_test_truck_position(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // DRONECOURSE_TESTSUITE_H
