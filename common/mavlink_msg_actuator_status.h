#pragma once
// MESSAGE ACTUATOR_STATUS PACKING

#define MAVLINK_MSG_ID_ACTUATOR_STATUS 180


typedef struct __mavlink_actuator_status_t {
 float actuator_1; /*<  Value for linear actuator 1*/
 float actuator_2; /*<  Value for linear actuator 2*/
} mavlink_actuator_status_t;

#define MAVLINK_MSG_ID_ACTUATOR_STATUS_LEN 8
#define MAVLINK_MSG_ID_ACTUATOR_STATUS_MIN_LEN 8
#define MAVLINK_MSG_ID_180_LEN 8
#define MAVLINK_MSG_ID_180_MIN_LEN 8

#define MAVLINK_MSG_ID_ACTUATOR_STATUS_CRC 130
#define MAVLINK_MSG_ID_180_CRC 130



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ACTUATOR_STATUS { \
    180, \
    "ACTUATOR_STATUS", \
    2, \
    {  { "actuator_1", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_actuator_status_t, actuator_1) }, \
         { "actuator_2", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_actuator_status_t, actuator_2) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ACTUATOR_STATUS { \
    "ACTUATOR_STATUS", \
    2, \
    {  { "actuator_1", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_actuator_status_t, actuator_1) }, \
         { "actuator_2", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_actuator_status_t, actuator_2) }, \
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
 * @param actuator_2  Value for linear actuator 2
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_actuator_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float actuator_1, float actuator_2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ACTUATOR_STATUS_LEN];
    _mav_put_float(buf, 0, actuator_1);
    _mav_put_float(buf, 4, actuator_2);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ACTUATOR_STATUS_LEN);
#else
    mavlink_actuator_status_t packet;
    packet.actuator_1 = actuator_1;
    packet.actuator_2 = actuator_2;

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
 * @param actuator_2  Value for linear actuator 2
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_actuator_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float actuator_1,float actuator_2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ACTUATOR_STATUS_LEN];
    _mav_put_float(buf, 0, actuator_1);
    _mav_put_float(buf, 4, actuator_2);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ACTUATOR_STATUS_LEN);
#else
    mavlink_actuator_status_t packet;
    packet.actuator_1 = actuator_1;
    packet.actuator_2 = actuator_2;

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
    return mavlink_msg_actuator_status_pack(system_id, component_id, msg, actuator_status->actuator_1, actuator_status->actuator_2);
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
    return mavlink_msg_actuator_status_pack_chan(system_id, component_id, chan, msg, actuator_status->actuator_1, actuator_status->actuator_2);
}

/**
 * @brief Send a actuator_status message
 * @param chan MAVLink channel to send the message
 *
 * @param actuator_1  Value for linear actuator 1
 * @param actuator_2  Value for linear actuator 2
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_actuator_status_send(mavlink_channel_t chan, float actuator_1, float actuator_2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ACTUATOR_STATUS_LEN];
    _mav_put_float(buf, 0, actuator_1);
    _mav_put_float(buf, 4, actuator_2);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACTUATOR_STATUS, buf, MAVLINK_MSG_ID_ACTUATOR_STATUS_MIN_LEN, MAVLINK_MSG_ID_ACTUATOR_STATUS_LEN, MAVLINK_MSG_ID_ACTUATOR_STATUS_CRC);
#else
    mavlink_actuator_status_t packet;
    packet.actuator_1 = actuator_1;
    packet.actuator_2 = actuator_2;

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
    mavlink_msg_actuator_status_send(chan, actuator_status->actuator_1, actuator_status->actuator_2);
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
static inline void mavlink_msg_actuator_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float actuator_1, float actuator_2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, actuator_1);
    _mav_put_float(buf, 4, actuator_2);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACTUATOR_STATUS, buf, MAVLINK_MSG_ID_ACTUATOR_STATUS_MIN_LEN, MAVLINK_MSG_ID_ACTUATOR_STATUS_LEN, MAVLINK_MSG_ID_ACTUATOR_STATUS_CRC);
#else
    mavlink_actuator_status_t *packet = (mavlink_actuator_status_t *)msgbuf;
    packet->actuator_1 = actuator_1;
    packet->actuator_2 = actuator_2;

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
 * @brief Get field actuator_2 from actuator_status message
 *
 * @return  Value for linear actuator 2
 */
static inline float mavlink_msg_actuator_status_get_actuator_2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
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
    actuator_status->actuator_2 = mavlink_msg_actuator_status_get_actuator_2(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ACTUATOR_STATUS_LEN? msg->len : MAVLINK_MSG_ID_ACTUATOR_STATUS_LEN;
        memset(actuator_status, 0, MAVLINK_MSG_ID_ACTUATOR_STATUS_LEN);
    memcpy(actuator_status, _MAV_PAYLOAD(msg), len);
#endif
}
