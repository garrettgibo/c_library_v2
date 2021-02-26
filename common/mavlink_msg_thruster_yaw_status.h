#pragma once
// MESSAGE THRUSTER_YAW_STATUS PACKING

#define MAVLINK_MSG_ID_THRUSTER_YAW_STATUS 184


typedef struct __mavlink_thruster_yaw_status_t {
 float thruster_yaw_1; /*<  Values for cold gas yaw thruster 1*/
 float thruster_yaw_2; /*<  Values for cold gas yaw thruster 2*/
} mavlink_thruster_yaw_status_t;

#define MAVLINK_MSG_ID_THRUSTER_YAW_STATUS_LEN 8
#define MAVLINK_MSG_ID_THRUSTER_YAW_STATUS_MIN_LEN 8
#define MAVLINK_MSG_ID_184_LEN 8
#define MAVLINK_MSG_ID_184_MIN_LEN 8

#define MAVLINK_MSG_ID_THRUSTER_YAW_STATUS_CRC 73
#define MAVLINK_MSG_ID_184_CRC 73



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_THRUSTER_YAW_STATUS { \
    184, \
    "THRUSTER_YAW_STATUS", \
    2, \
    {  { "thruster_yaw_1", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_thruster_yaw_status_t, thruster_yaw_1) }, \
         { "thruster_yaw_2", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_thruster_yaw_status_t, thruster_yaw_2) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_THRUSTER_YAW_STATUS { \
    "THRUSTER_YAW_STATUS", \
    2, \
    {  { "thruster_yaw_1", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_thruster_yaw_status_t, thruster_yaw_1) }, \
         { "thruster_yaw_2", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_thruster_yaw_status_t, thruster_yaw_2) }, \
         } \
}
#endif

/**
 * @brief Pack a thruster_yaw_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param thruster_yaw_1  Values for cold gas yaw thruster 1
 * @param thruster_yaw_2  Values for cold gas yaw thruster 2
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_thruster_yaw_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float thruster_yaw_1, float thruster_yaw_2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_THRUSTER_YAW_STATUS_LEN];
    _mav_put_float(buf, 0, thruster_yaw_1);
    _mav_put_float(buf, 4, thruster_yaw_2);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_THRUSTER_YAW_STATUS_LEN);
#else
    mavlink_thruster_yaw_status_t packet;
    packet.thruster_yaw_1 = thruster_yaw_1;
    packet.thruster_yaw_2 = thruster_yaw_2;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_THRUSTER_YAW_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_THRUSTER_YAW_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_THRUSTER_YAW_STATUS_MIN_LEN, MAVLINK_MSG_ID_THRUSTER_YAW_STATUS_LEN, MAVLINK_MSG_ID_THRUSTER_YAW_STATUS_CRC);
}

/**
 * @brief Pack a thruster_yaw_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param thruster_yaw_1  Values for cold gas yaw thruster 1
 * @param thruster_yaw_2  Values for cold gas yaw thruster 2
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_thruster_yaw_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float thruster_yaw_1,float thruster_yaw_2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_THRUSTER_YAW_STATUS_LEN];
    _mav_put_float(buf, 0, thruster_yaw_1);
    _mav_put_float(buf, 4, thruster_yaw_2);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_THRUSTER_YAW_STATUS_LEN);
#else
    mavlink_thruster_yaw_status_t packet;
    packet.thruster_yaw_1 = thruster_yaw_1;
    packet.thruster_yaw_2 = thruster_yaw_2;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_THRUSTER_YAW_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_THRUSTER_YAW_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_THRUSTER_YAW_STATUS_MIN_LEN, MAVLINK_MSG_ID_THRUSTER_YAW_STATUS_LEN, MAVLINK_MSG_ID_THRUSTER_YAW_STATUS_CRC);
}

/**
 * @brief Encode a thruster_yaw_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param thruster_yaw_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_thruster_yaw_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_thruster_yaw_status_t* thruster_yaw_status)
{
    return mavlink_msg_thruster_yaw_status_pack(system_id, component_id, msg, thruster_yaw_status->thruster_yaw_1, thruster_yaw_status->thruster_yaw_2);
}

/**
 * @brief Encode a thruster_yaw_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param thruster_yaw_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_thruster_yaw_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_thruster_yaw_status_t* thruster_yaw_status)
{
    return mavlink_msg_thruster_yaw_status_pack_chan(system_id, component_id, chan, msg, thruster_yaw_status->thruster_yaw_1, thruster_yaw_status->thruster_yaw_2);
}

/**
 * @brief Send a thruster_yaw_status message
 * @param chan MAVLink channel to send the message
 *
 * @param thruster_yaw_1  Values for cold gas yaw thruster 1
 * @param thruster_yaw_2  Values for cold gas yaw thruster 2
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_thruster_yaw_status_send(mavlink_channel_t chan, float thruster_yaw_1, float thruster_yaw_2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_THRUSTER_YAW_STATUS_LEN];
    _mav_put_float(buf, 0, thruster_yaw_1);
    _mav_put_float(buf, 4, thruster_yaw_2);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_THRUSTER_YAW_STATUS, buf, MAVLINK_MSG_ID_THRUSTER_YAW_STATUS_MIN_LEN, MAVLINK_MSG_ID_THRUSTER_YAW_STATUS_LEN, MAVLINK_MSG_ID_THRUSTER_YAW_STATUS_CRC);
#else
    mavlink_thruster_yaw_status_t packet;
    packet.thruster_yaw_1 = thruster_yaw_1;
    packet.thruster_yaw_2 = thruster_yaw_2;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_THRUSTER_YAW_STATUS, (const char *)&packet, MAVLINK_MSG_ID_THRUSTER_YAW_STATUS_MIN_LEN, MAVLINK_MSG_ID_THRUSTER_YAW_STATUS_LEN, MAVLINK_MSG_ID_THRUSTER_YAW_STATUS_CRC);
#endif
}

/**
 * @brief Send a thruster_yaw_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_thruster_yaw_status_send_struct(mavlink_channel_t chan, const mavlink_thruster_yaw_status_t* thruster_yaw_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_thruster_yaw_status_send(chan, thruster_yaw_status->thruster_yaw_1, thruster_yaw_status->thruster_yaw_2);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_THRUSTER_YAW_STATUS, (const char *)thruster_yaw_status, MAVLINK_MSG_ID_THRUSTER_YAW_STATUS_MIN_LEN, MAVLINK_MSG_ID_THRUSTER_YAW_STATUS_LEN, MAVLINK_MSG_ID_THRUSTER_YAW_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_THRUSTER_YAW_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_thruster_yaw_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float thruster_yaw_1, float thruster_yaw_2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, thruster_yaw_1);
    _mav_put_float(buf, 4, thruster_yaw_2);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_THRUSTER_YAW_STATUS, buf, MAVLINK_MSG_ID_THRUSTER_YAW_STATUS_MIN_LEN, MAVLINK_MSG_ID_THRUSTER_YAW_STATUS_LEN, MAVLINK_MSG_ID_THRUSTER_YAW_STATUS_CRC);
#else
    mavlink_thruster_yaw_status_t *packet = (mavlink_thruster_yaw_status_t *)msgbuf;
    packet->thruster_yaw_1 = thruster_yaw_1;
    packet->thruster_yaw_2 = thruster_yaw_2;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_THRUSTER_YAW_STATUS, (const char *)packet, MAVLINK_MSG_ID_THRUSTER_YAW_STATUS_MIN_LEN, MAVLINK_MSG_ID_THRUSTER_YAW_STATUS_LEN, MAVLINK_MSG_ID_THRUSTER_YAW_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE THRUSTER_YAW_STATUS UNPACKING


/**
 * @brief Get field thruster_yaw_1 from thruster_yaw_status message
 *
 * @return  Values for cold gas yaw thruster 1
 */
static inline float mavlink_msg_thruster_yaw_status_get_thruster_yaw_1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field thruster_yaw_2 from thruster_yaw_status message
 *
 * @return  Values for cold gas yaw thruster 2
 */
static inline float mavlink_msg_thruster_yaw_status_get_thruster_yaw_2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Decode a thruster_yaw_status message into a struct
 *
 * @param msg The message to decode
 * @param thruster_yaw_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_thruster_yaw_status_decode(const mavlink_message_t* msg, mavlink_thruster_yaw_status_t* thruster_yaw_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    thruster_yaw_status->thruster_yaw_1 = mavlink_msg_thruster_yaw_status_get_thruster_yaw_1(msg);
    thruster_yaw_status->thruster_yaw_2 = mavlink_msg_thruster_yaw_status_get_thruster_yaw_2(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_THRUSTER_YAW_STATUS_LEN? msg->len : MAVLINK_MSG_ID_THRUSTER_YAW_STATUS_LEN;
        memset(thruster_yaw_status, 0, MAVLINK_MSG_ID_THRUSTER_YAW_STATUS_LEN);
    memcpy(thruster_yaw_status, _MAV_PAYLOAD(msg), len);
#endif
}
