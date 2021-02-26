#pragma once
// MESSAGE ROLL_PITCH_STATUS PACKING

#define MAVLINK_MSG_ID_ROLL_PITCH_STATUS 182


typedef struct __mavlink_roll_pitch_status_t {
 float roll_target; /*<  Value for roll target*/
 float pitch_target; /*<  Value for pitch target*/
} mavlink_roll_pitch_status_t;

#define MAVLINK_MSG_ID_ROLL_PITCH_STATUS_LEN 8
#define MAVLINK_MSG_ID_ROLL_PITCH_STATUS_MIN_LEN 8
#define MAVLINK_MSG_ID_182_LEN 8
#define MAVLINK_MSG_ID_182_MIN_LEN 8

#define MAVLINK_MSG_ID_ROLL_PITCH_STATUS_CRC 99
#define MAVLINK_MSG_ID_182_CRC 99



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ROLL_PITCH_STATUS { \
    182, \
    "ROLL_PITCH_STATUS", \
    2, \
    {  { "roll_target", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_roll_pitch_status_t, roll_target) }, \
         { "pitch_target", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_roll_pitch_status_t, pitch_target) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ROLL_PITCH_STATUS { \
    "ROLL_PITCH_STATUS", \
    2, \
    {  { "roll_target", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_roll_pitch_status_t, roll_target) }, \
         { "pitch_target", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_roll_pitch_status_t, pitch_target) }, \
         } \
}
#endif

/**
 * @brief Pack a roll_pitch_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param roll_target  Value for roll target
 * @param pitch_target  Value for pitch target
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_roll_pitch_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float roll_target, float pitch_target)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROLL_PITCH_STATUS_LEN];
    _mav_put_float(buf, 0, roll_target);
    _mav_put_float(buf, 4, pitch_target);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROLL_PITCH_STATUS_LEN);
#else
    mavlink_roll_pitch_status_t packet;
    packet.roll_target = roll_target;
    packet.pitch_target = pitch_target;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ROLL_PITCH_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ROLL_PITCH_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ROLL_PITCH_STATUS_MIN_LEN, MAVLINK_MSG_ID_ROLL_PITCH_STATUS_LEN, MAVLINK_MSG_ID_ROLL_PITCH_STATUS_CRC);
}

/**
 * @brief Pack a roll_pitch_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param roll_target  Value for roll target
 * @param pitch_target  Value for pitch target
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_roll_pitch_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float roll_target,float pitch_target)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROLL_PITCH_STATUS_LEN];
    _mav_put_float(buf, 0, roll_target);
    _mav_put_float(buf, 4, pitch_target);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROLL_PITCH_STATUS_LEN);
#else
    mavlink_roll_pitch_status_t packet;
    packet.roll_target = roll_target;
    packet.pitch_target = pitch_target;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ROLL_PITCH_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ROLL_PITCH_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ROLL_PITCH_STATUS_MIN_LEN, MAVLINK_MSG_ID_ROLL_PITCH_STATUS_LEN, MAVLINK_MSG_ID_ROLL_PITCH_STATUS_CRC);
}

/**
 * @brief Encode a roll_pitch_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param roll_pitch_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_roll_pitch_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_roll_pitch_status_t* roll_pitch_status)
{
    return mavlink_msg_roll_pitch_status_pack(system_id, component_id, msg, roll_pitch_status->roll_target, roll_pitch_status->pitch_target);
}

/**
 * @brief Encode a roll_pitch_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param roll_pitch_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_roll_pitch_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_roll_pitch_status_t* roll_pitch_status)
{
    return mavlink_msg_roll_pitch_status_pack_chan(system_id, component_id, chan, msg, roll_pitch_status->roll_target, roll_pitch_status->pitch_target);
}

/**
 * @brief Send a roll_pitch_status message
 * @param chan MAVLink channel to send the message
 *
 * @param roll_target  Value for roll target
 * @param pitch_target  Value for pitch target
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_roll_pitch_status_send(mavlink_channel_t chan, float roll_target, float pitch_target)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROLL_PITCH_STATUS_LEN];
    _mav_put_float(buf, 0, roll_target);
    _mav_put_float(buf, 4, pitch_target);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROLL_PITCH_STATUS, buf, MAVLINK_MSG_ID_ROLL_PITCH_STATUS_MIN_LEN, MAVLINK_MSG_ID_ROLL_PITCH_STATUS_LEN, MAVLINK_MSG_ID_ROLL_PITCH_STATUS_CRC);
#else
    mavlink_roll_pitch_status_t packet;
    packet.roll_target = roll_target;
    packet.pitch_target = pitch_target;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROLL_PITCH_STATUS, (const char *)&packet, MAVLINK_MSG_ID_ROLL_PITCH_STATUS_MIN_LEN, MAVLINK_MSG_ID_ROLL_PITCH_STATUS_LEN, MAVLINK_MSG_ID_ROLL_PITCH_STATUS_CRC);
#endif
}

/**
 * @brief Send a roll_pitch_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_roll_pitch_status_send_struct(mavlink_channel_t chan, const mavlink_roll_pitch_status_t* roll_pitch_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_roll_pitch_status_send(chan, roll_pitch_status->roll_target, roll_pitch_status->pitch_target);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROLL_PITCH_STATUS, (const char *)roll_pitch_status, MAVLINK_MSG_ID_ROLL_PITCH_STATUS_MIN_LEN, MAVLINK_MSG_ID_ROLL_PITCH_STATUS_LEN, MAVLINK_MSG_ID_ROLL_PITCH_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_ROLL_PITCH_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_roll_pitch_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float roll_target, float pitch_target)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, roll_target);
    _mav_put_float(buf, 4, pitch_target);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROLL_PITCH_STATUS, buf, MAVLINK_MSG_ID_ROLL_PITCH_STATUS_MIN_LEN, MAVLINK_MSG_ID_ROLL_PITCH_STATUS_LEN, MAVLINK_MSG_ID_ROLL_PITCH_STATUS_CRC);
#else
    mavlink_roll_pitch_status_t *packet = (mavlink_roll_pitch_status_t *)msgbuf;
    packet->roll_target = roll_target;
    packet->pitch_target = pitch_target;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROLL_PITCH_STATUS, (const char *)packet, MAVLINK_MSG_ID_ROLL_PITCH_STATUS_MIN_LEN, MAVLINK_MSG_ID_ROLL_PITCH_STATUS_LEN, MAVLINK_MSG_ID_ROLL_PITCH_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE ROLL_PITCH_STATUS UNPACKING


/**
 * @brief Get field roll_target from roll_pitch_status message
 *
 * @return  Value for roll target
 */
static inline float mavlink_msg_roll_pitch_status_get_roll_target(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field pitch_target from roll_pitch_status message
 *
 * @return  Value for pitch target
 */
static inline float mavlink_msg_roll_pitch_status_get_pitch_target(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Decode a roll_pitch_status message into a struct
 *
 * @param msg The message to decode
 * @param roll_pitch_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_roll_pitch_status_decode(const mavlink_message_t* msg, mavlink_roll_pitch_status_t* roll_pitch_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    roll_pitch_status->roll_target = mavlink_msg_roll_pitch_status_get_roll_target(msg);
    roll_pitch_status->pitch_target = mavlink_msg_roll_pitch_status_get_pitch_target(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ROLL_PITCH_STATUS_LEN? msg->len : MAVLINK_MSG_ID_ROLL_PITCH_STATUS_LEN;
        memset(roll_pitch_status, 0, MAVLINK_MSG_ID_ROLL_PITCH_STATUS_LEN);
    memcpy(roll_pitch_status, _MAV_PAYLOAD(msg), len);
#endif
}
