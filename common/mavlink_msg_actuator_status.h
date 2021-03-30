#pragma once
// MESSAGE ACTUATOR_STATUS PACKING

#define MAVLINK_MSG_ID_ACTUATOR_STATUS 180


typedef struct __mavlink_actuator_status_t {
 float actuator_1; /*<  Value for linear actuator 1*/
 float actuator_1_target; /*<  Value for linear actuator 1 target*/
 float actuator_1_current; /*<  Value for linear actuator 1 current*/
 float actuator_1_velocity; /*<  Value for linear actuator 1 velocity*/
 float actuator_2; /*<  Value for linear actuator 2*/
 float actuator_2_target; /*<  Value for linear actuator 2 target*/
 float actuator_2_current; /*<  Value for linear actuator 2 current*/
 float actuator_2_velocity; /*<  Value for linear actuator 2 velocity*/
} mavlink_actuator_status_t;

#define MAVLINK_MSG_ID_ACTUATOR_STATUS_LEN 32
#define MAVLINK_MSG_ID_ACTUATOR_STATUS_MIN_LEN 32
#define MAVLINK_MSG_ID_180_LEN 32
#define MAVLINK_MSG_ID_180_MIN_LEN 32

#define MAVLINK_MSG_ID_ACTUATOR_STATUS_CRC 116
#define MAVLINK_MSG_ID_180_CRC 116



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ACTUATOR_STATUS { \
    180, \
    "ACTUATOR_STATUS", \
    8, \
    {  { "actuator_1", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_actuator_status_t, actuator_1) }, \
         { "actuator_1_target", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_actuator_status_t, actuator_1_target) }, \
         { "actuator_1_current", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_actuator_status_t, actuator_1_current) }, \
         { "actuator_1_velocity", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_actuator_status_t, actuator_1_velocity) }, \
         { "actuator_2", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_actuator_status_t, actuator_2) }, \
         { "actuator_2_target", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_actuator_status_t, actuator_2_target) }, \
         { "actuator_2_current", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_actuator_status_t, actuator_2_current) }, \
         { "actuator_2_velocity", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_actuator_status_t, actuator_2_velocity) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ACTUATOR_STATUS { \
    "ACTUATOR_STATUS", \
    8, \
    {  { "actuator_1", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_actuator_status_t, actuator_1) }, \
         { "actuator_1_target", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_actuator_status_t, actuator_1_target) }, \
         { "actuator_1_current", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_actuator_status_t, actuator_1_current) }, \
         { "actuator_1_velocity", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_actuator_status_t, actuator_1_velocity) }, \
         { "actuator_2", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_actuator_status_t, actuator_2) }, \
         { "actuator_2_target", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_actuator_status_t, actuator_2_target) }, \
         { "actuator_2_current", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_actuator_status_t, actuator_2_current) }, \
         { "actuator_2_velocity", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_actuator_status_t, actuator_2_velocity) }, \
         } \
}
#endif

/**
 * @brief Pack a actuator_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param actuator_1  Value for linear actuator 1
 * @param actuator_1_target  Value for linear actuator 1 target
 * @param actuator_1_current  Value for linear actuator 1 current
 * @param actuator_1_velocity  Value for linear actuator 1 velocity
 * @param actuator_2  Value for linear actuator 2
 * @param actuator_2_target  Value for linear actuator 2 target
 * @param actuator_2_current  Value for linear actuator 2 current
 * @param actuator_2_velocity  Value for linear actuator 2 velocity
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_actuator_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float actuator_1, float actuator_1_target, float actuator_1_current, float actuator_1_velocity, float actuator_2, float actuator_2_target, float actuator_2_current, float actuator_2_velocity)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ACTUATOR_STATUS_LEN];
    _mav_put_float(buf, 0, actuator_1);
    _mav_put_float(buf, 4, actuator_1_target);
    _mav_put_float(buf, 8, actuator_1_current);
    _mav_put_float(buf, 12, actuator_1_velocity);
    _mav_put_float(buf, 16, actuator_2);
    _mav_put_float(buf, 20, actuator_2_target);
    _mav_put_float(buf, 24, actuator_2_current);
    _mav_put_float(buf, 28, actuator_2_velocity);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ACTUATOR_STATUS_LEN);
#else
    mavlink_actuator_status_t packet;
    packet.actuator_1 = actuator_1;
    packet.actuator_1_target = actuator_1_target;
    packet.actuator_1_current = actuator_1_current;
    packet.actuator_1_velocity = actuator_1_velocity;
    packet.actuator_2 = actuator_2;
    packet.actuator_2_target = actuator_2_target;
    packet.actuator_2_current = actuator_2_current;
    packet.actuator_2_velocity = actuator_2_velocity;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ACTUATOR_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ACTUATOR_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ACTUATOR_STATUS_MIN_LEN, MAVLINK_MSG_ID_ACTUATOR_STATUS_LEN, MAVLINK_MSG_ID_ACTUATOR_STATUS_CRC);
}

/**
 * @brief Pack a actuator_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param actuator_1  Value for linear actuator 1
 * @param actuator_1_target  Value for linear actuator 1 target
 * @param actuator_1_current  Value for linear actuator 1 current
 * @param actuator_1_velocity  Value for linear actuator 1 velocity
 * @param actuator_2  Value for linear actuator 2
 * @param actuator_2_target  Value for linear actuator 2 target
 * @param actuator_2_current  Value for linear actuator 2 current
 * @param actuator_2_velocity  Value for linear actuator 2 velocity
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_actuator_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float actuator_1,float actuator_1_target,float actuator_1_current,float actuator_1_velocity,float actuator_2,float actuator_2_target,float actuator_2_current,float actuator_2_velocity)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ACTUATOR_STATUS_LEN];
    _mav_put_float(buf, 0, actuator_1);
    _mav_put_float(buf, 4, actuator_1_target);
    _mav_put_float(buf, 8, actuator_1_current);
    _mav_put_float(buf, 12, actuator_1_velocity);
    _mav_put_float(buf, 16, actuator_2);
    _mav_put_float(buf, 20, actuator_2_target);
    _mav_put_float(buf, 24, actuator_2_current);
    _mav_put_float(buf, 28, actuator_2_velocity);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ACTUATOR_STATUS_LEN);
#else
    mavlink_actuator_status_t packet;
    packet.actuator_1 = actuator_1;
    packet.actuator_1_target = actuator_1_target;
    packet.actuator_1_current = actuator_1_current;
    packet.actuator_1_velocity = actuator_1_velocity;
    packet.actuator_2 = actuator_2;
    packet.actuator_2_target = actuator_2_target;
    packet.actuator_2_current = actuator_2_current;
    packet.actuator_2_velocity = actuator_2_velocity;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ACTUATOR_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ACTUATOR_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ACTUATOR_STATUS_MIN_LEN, MAVLINK_MSG_ID_ACTUATOR_STATUS_LEN, MAVLINK_MSG_ID_ACTUATOR_STATUS_CRC);
}

/**
 * @brief Encode a actuator_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param actuator_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_actuator_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_actuator_status_t* actuator_status)
{
    return mavlink_msg_actuator_status_pack(system_id, component_id, msg, actuator_status->actuator_1, actuator_status->actuator_1_target, actuator_status->actuator_1_current, actuator_status->actuator_1_velocity, actuator_status->actuator_2, actuator_status->actuator_2_target, actuator_status->actuator_2_current, actuator_status->actuator_2_velocity);
}

/**
 * @brief Encode a actuator_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param actuator_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_actuator_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_actuator_status_t* actuator_status)
{
    return mavlink_msg_actuator_status_pack_chan(system_id, component_id, chan, msg, actuator_status->actuator_1, actuator_status->actuator_1_target, actuator_status->actuator_1_current, actuator_status->actuator_1_velocity, actuator_status->actuator_2, actuator_status->actuator_2_target, actuator_status->actuator_2_current, actuator_status->actuator_2_velocity);
}

/**
 * @brief Send a actuator_status message
 * @param chan MAVLink channel to send the message
 *
 * @param actuator_1  Value for linear actuator 1
 * @param actuator_1_target  Value for linear actuator 1 target
 * @param actuator_1_current  Value for linear actuator 1 current
 * @param actuator_1_velocity  Value for linear actuator 1 velocity
 * @param actuator_2  Value for linear actuator 2
 * @param actuator_2_target  Value for linear actuator 2 target
 * @param actuator_2_current  Value for linear actuator 2 current
 * @param actuator_2_velocity  Value for linear actuator 2 velocity
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_actuator_status_send(mavlink_channel_t chan, float actuator_1, float actuator_1_target, float actuator_1_current, float actuator_1_velocity, float actuator_2, float actuator_2_target, float actuator_2_current, float actuator_2_velocity)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ACTUATOR_STATUS_LEN];
    _mav_put_float(buf, 0, actuator_1);
    _mav_put_float(buf, 4, actuator_1_target);
    _mav_put_float(buf, 8, actuator_1_current);
    _mav_put_float(buf, 12, actuator_1_velocity);
    _mav_put_float(buf, 16, actuator_2);
    _mav_put_float(buf, 20, actuator_2_target);
    _mav_put_float(buf, 24, actuator_2_current);
    _mav_put_float(buf, 28, actuator_2_velocity);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACTUATOR_STATUS, buf, MAVLINK_MSG_ID_ACTUATOR_STATUS_MIN_LEN, MAVLINK_MSG_ID_ACTUATOR_STATUS_LEN, MAVLINK_MSG_ID_ACTUATOR_STATUS_CRC);
#else
    mavlink_actuator_status_t packet;
    packet.actuator_1 = actuator_1;
    packet.actuator_1_target = actuator_1_target;
    packet.actuator_1_current = actuator_1_current;
    packet.actuator_1_velocity = actuator_1_velocity;
    packet.actuator_2 = actuator_2;
    packet.actuator_2_target = actuator_2_target;
    packet.actuator_2_current = actuator_2_current;
    packet.actuator_2_velocity = actuator_2_velocity;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACTUATOR_STATUS, (const char *)&packet, MAVLINK_MSG_ID_ACTUATOR_STATUS_MIN_LEN, MAVLINK_MSG_ID_ACTUATOR_STATUS_LEN, MAVLINK_MSG_ID_ACTUATOR_STATUS_CRC);
#endif
}

/**
 * @brief Send a actuator_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_actuator_status_send_struct(mavlink_channel_t chan, const mavlink_actuator_status_t* actuator_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_actuator_status_send(chan, actuator_status->actuator_1, actuator_status->actuator_1_target, actuator_status->actuator_1_current, actuator_status->actuator_1_velocity, actuator_status->actuator_2, actuator_status->actuator_2_target, actuator_status->actuator_2_current, actuator_status->actuator_2_velocity);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACTUATOR_STATUS, (const char *)actuator_status, MAVLINK_MSG_ID_ACTUATOR_STATUS_MIN_LEN, MAVLINK_MSG_ID_ACTUATOR_STATUS_LEN, MAVLINK_MSG_ID_ACTUATOR_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_ACTUATOR_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_actuator_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float actuator_1, float actuator_1_target, float actuator_1_current, float actuator_1_velocity, float actuator_2, float actuator_2_target, float actuator_2_current, float actuator_2_velocity)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, actuator_1);
    _mav_put_float(buf, 4, actuator_1_target);
    _mav_put_float(buf, 8, actuator_1_current);
    _mav_put_float(buf, 12, actuator_1_velocity);
    _mav_put_float(buf, 16, actuator_2);
    _mav_put_float(buf, 20, actuator_2_target);
    _mav_put_float(buf, 24, actuator_2_current);
    _mav_put_float(buf, 28, actuator_2_velocity);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACTUATOR_STATUS, buf, MAVLINK_MSG_ID_ACTUATOR_STATUS_MIN_LEN, MAVLINK_MSG_ID_ACTUATOR_STATUS_LEN, MAVLINK_MSG_ID_ACTUATOR_STATUS_CRC);
#else
    mavlink_actuator_status_t *packet = (mavlink_actuator_status_t *)msgbuf;
    packet->actuator_1 = actuator_1;
    packet->actuator_1_target = actuator_1_target;
    packet->actuator_1_current = actuator_1_current;
    packet->actuator_1_velocity = actuator_1_velocity;
    packet->actuator_2 = actuator_2;
    packet->actuator_2_target = actuator_2_target;
    packet->actuator_2_current = actuator_2_current;
    packet->actuator_2_velocity = actuator_2_velocity;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACTUATOR_STATUS, (const char *)packet, MAVLINK_MSG_ID_ACTUATOR_STATUS_MIN_LEN, MAVLINK_MSG_ID_ACTUATOR_STATUS_LEN, MAVLINK_MSG_ID_ACTUATOR_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE ACTUATOR_STATUS UNPACKING


/**
 * @brief Get field actuator_1 from actuator_status message
 *
 * @return  Value for linear actuator 1
 */
static inline float mavlink_msg_actuator_status_get_actuator_1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field actuator_1_target from actuator_status message
 *
 * @return  Value for linear actuator 1 target
 */
static inline float mavlink_msg_actuator_status_get_actuator_1_target(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field actuator_1_current from actuator_status message
 *
 * @return  Value for linear actuator 1 current
 */
static inline float mavlink_msg_actuator_status_get_actuator_1_current(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field actuator_1_velocity from actuator_status message
 *
 * @return  Value for linear actuator 1 velocity
 */
static inline float mavlink_msg_actuator_status_get_actuator_1_velocity(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field actuator_2 from actuator_status message
 *
 * @return  Value for linear actuator 2
 */
static inline float mavlink_msg_actuator_status_get_actuator_2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field actuator_2_target from actuator_status message
 *
 * @return  Value for linear actuator 2 target
 */
static inline float mavlink_msg_actuator_status_get_actuator_2_target(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field actuator_2_current from actuator_status message
 *
 * @return  Value for linear actuator 2 current
 */
static inline float mavlink_msg_actuator_status_get_actuator_2_current(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field actuator_2_velocity from actuator_status message
 *
 * @return  Value for linear actuator 2 velocity
 */
static inline float mavlink_msg_actuator_status_get_actuator_2_velocity(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Decode a actuator_status message into a struct
 *
 * @param msg The message to decode
 * @param actuator_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_actuator_status_decode(const mavlink_message_t* msg, mavlink_actuator_status_t* actuator_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    actuator_status->actuator_1 = mavlink_msg_actuator_status_get_actuator_1(msg);
    actuator_status->actuator_1_target = mavlink_msg_actuator_status_get_actuator_1_target(msg);
    actuator_status->actuator_1_current = mavlink_msg_actuator_status_get_actuator_1_current(msg);
    actuator_status->actuator_1_velocity = mavlink_msg_actuator_status_get_actuator_1_velocity(msg);
    actuator_status->actuator_2 = mavlink_msg_actuator_status_get_actuator_2(msg);
    actuator_status->actuator_2_target = mavlink_msg_actuator_status_get_actuator_2_target(msg);
    actuator_status->actuator_2_current = mavlink_msg_actuator_status_get_actuator_2_current(msg);
    actuator_status->actuator_2_velocity = mavlink_msg_actuator_status_get_actuator_2_velocity(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ACTUATOR_STATUS_LEN? msg->len : MAVLINK_MSG_ID_ACTUATOR_STATUS_LEN;
        memset(actuator_status, 0, MAVLINK_MSG_ID_ACTUATOR_STATUS_LEN);
    memcpy(actuator_status, _MAV_PAYLOAD(msg), len);
#endif
}
