#pragma once
// MESSAGE ROLL_PITCH_SETPOINT PACKING

#define MAVLINK_MSG_ID_ROLL_PITCH_SETPOINT 185


typedef struct __mavlink_roll_pitch_setpoint_t {
 float roll_setpoint; /*<  Roll Setpoint*/
 float pitch_setpoint; /*<  Pitch Setpoint*/
} mavlink_roll_pitch_setpoint_t;

#define MAVLINK_MSG_ID_ROLL_PITCH_SETPOINT_LEN 8
#define MAVLINK_MSG_ID_ROLL_PITCH_SETPOINT_MIN_LEN 8
#define MAVLINK_MSG_ID_185_LEN 8
#define MAVLINK_MSG_ID_185_MIN_LEN 8

#define MAVLINK_MSG_ID_ROLL_PITCH_SETPOINT_CRC 82
#define MAVLINK_MSG_ID_185_CRC 82



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ROLL_PITCH_SETPOINT { \
    185, \
    "ROLL_PITCH_SETPOINT", \
    2, \
    {  { "roll_setpoint", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_roll_pitch_setpoint_t, roll_setpoint) }, \
         { "pitch_setpoint", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_roll_pitch_setpoint_t, pitch_setpoint) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ROLL_PITCH_SETPOINT { \
    "ROLL_PITCH_SETPOINT", \
    2, \
    {  { "roll_setpoint", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_roll_pitch_setpoint_t, roll_setpoint) }, \
         { "pitch_setpoint", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_roll_pitch_setpoint_t, pitch_setpoint) }, \
         } \
}
#endif

/**
 * @brief Pack a roll_pitch_setpoint message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param roll_setpoint  Roll Setpoint
 * @param pitch_setpoint  Pitch Setpoint
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_roll_pitch_setpoint_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float roll_setpoint, float pitch_setpoint)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROLL_PITCH_SETPOINT_LEN];
    _mav_put_float(buf, 0, roll_setpoint);
    _mav_put_float(buf, 4, pitch_setpoint);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROLL_PITCH_SETPOINT_LEN);
#else
    mavlink_roll_pitch_setpoint_t packet;
    packet.roll_setpoint = roll_setpoint;
    packet.pitch_setpoint = pitch_setpoint;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ROLL_PITCH_SETPOINT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ROLL_PITCH_SETPOINT;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ROLL_PITCH_SETPOINT_MIN_LEN, MAVLINK_MSG_ID_ROLL_PITCH_SETPOINT_LEN, MAVLINK_MSG_ID_ROLL_PITCH_SETPOINT_CRC);
}

/**
 * @brief Pack a roll_pitch_setpoint message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param roll_setpoint  Roll Setpoint
 * @param pitch_setpoint  Pitch Setpoint
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_roll_pitch_setpoint_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float roll_setpoint,float pitch_setpoint)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROLL_PITCH_SETPOINT_LEN];
    _mav_put_float(buf, 0, roll_setpoint);
    _mav_put_float(buf, 4, pitch_setpoint);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROLL_PITCH_SETPOINT_LEN);
#else
    mavlink_roll_pitch_setpoint_t packet;
    packet.roll_setpoint = roll_setpoint;
    packet.pitch_setpoint = pitch_setpoint;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ROLL_PITCH_SETPOINT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ROLL_PITCH_SETPOINT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ROLL_PITCH_SETPOINT_MIN_LEN, MAVLINK_MSG_ID_ROLL_PITCH_SETPOINT_LEN, MAVLINK_MSG_ID_ROLL_PITCH_SETPOINT_CRC);
}

/**
 * @brief Encode a roll_pitch_setpoint struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param roll_pitch_setpoint C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_roll_pitch_setpoint_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_roll_pitch_setpoint_t* roll_pitch_setpoint)
{
    return mavlink_msg_roll_pitch_setpoint_pack(system_id, component_id, msg, roll_pitch_setpoint->roll_setpoint, roll_pitch_setpoint->pitch_setpoint);
}

/**
 * @brief Encode a roll_pitch_setpoint struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param roll_pitch_setpoint C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_roll_pitch_setpoint_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_roll_pitch_setpoint_t* roll_pitch_setpoint)
{
    return mavlink_msg_roll_pitch_setpoint_pack_chan(system_id, component_id, chan, msg, roll_pitch_setpoint->roll_setpoint, roll_pitch_setpoint->pitch_setpoint);
}

/**
 * @brief Send a roll_pitch_setpoint message
 * @param chan MAVLink channel to send the message
 *
 * @param roll_setpoint  Roll Setpoint
 * @param pitch_setpoint  Pitch Setpoint
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_roll_pitch_setpoint_send(mavlink_channel_t chan, float roll_setpoint, float pitch_setpoint)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROLL_PITCH_SETPOINT_LEN];
    _mav_put_float(buf, 0, roll_setpoint);
    _mav_put_float(buf, 4, pitch_setpoint);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROLL_PITCH_SETPOINT, buf, MAVLINK_MSG_ID_ROLL_PITCH_SETPOINT_MIN_LEN, MAVLINK_MSG_ID_ROLL_PITCH_SETPOINT_LEN, MAVLINK_MSG_ID_ROLL_PITCH_SETPOINT_CRC);
#else
    mavlink_roll_pitch_setpoint_t packet;
    packet.roll_setpoint = roll_setpoint;
    packet.pitch_setpoint = pitch_setpoint;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROLL_PITCH_SETPOINT, (const char *)&packet, MAVLINK_MSG_ID_ROLL_PITCH_SETPOINT_MIN_LEN, MAVLINK_MSG_ID_ROLL_PITCH_SETPOINT_LEN, MAVLINK_MSG_ID_ROLL_PITCH_SETPOINT_CRC);
#endif
}

/**
 * @brief Send a roll_pitch_setpoint message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_roll_pitch_setpoint_send_struct(mavlink_channel_t chan, const mavlink_roll_pitch_setpoint_t* roll_pitch_setpoint)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_roll_pitch_setpoint_send(chan, roll_pitch_setpoint->roll_setpoint, roll_pitch_setpoint->pitch_setpoint);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROLL_PITCH_SETPOINT, (const char *)roll_pitch_setpoint, MAVLINK_MSG_ID_ROLL_PITCH_SETPOINT_MIN_LEN, MAVLINK_MSG_ID_ROLL_PITCH_SETPOINT_LEN, MAVLINK_MSG_ID_ROLL_PITCH_SETPOINT_CRC);
#endif
}

#if MAVLINK_MSG_ID_ROLL_PITCH_SETPOINT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_roll_pitch_setpoint_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float roll_setpoint, float pitch_setpoint)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, roll_setpoint);
    _mav_put_float(buf, 4, pitch_setpoint);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROLL_PITCH_SETPOINT, buf, MAVLINK_MSG_ID_ROLL_PITCH_SETPOINT_MIN_LEN, MAVLINK_MSG_ID_ROLL_PITCH_SETPOINT_LEN, MAVLINK_MSG_ID_ROLL_PITCH_SETPOINT_CRC);
#else
    mavlink_roll_pitch_setpoint_t *packet = (mavlink_roll_pitch_setpoint_t *)msgbuf;
    packet->roll_setpoint = roll_setpoint;
    packet->pitch_setpoint = pitch_setpoint;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROLL_PITCH_SETPOINT, (const char *)packet, MAVLINK_MSG_ID_ROLL_PITCH_SETPOINT_MIN_LEN, MAVLINK_MSG_ID_ROLL_PITCH_SETPOINT_LEN, MAVLINK_MSG_ID_ROLL_PITCH_SETPOINT_CRC);
#endif
}
#endif

#endif

// MESSAGE ROLL_PITCH_SETPOINT UNPACKING


/**
 * @brief Get field roll_setpoint from roll_pitch_setpoint message
 *
 * @return  Roll Setpoint
 */
static inline float mavlink_msg_roll_pitch_setpoint_get_roll_setpoint(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field pitch_setpoint from roll_pitch_setpoint message
 *
 * @return  Pitch Setpoint
 */
static inline float mavlink_msg_roll_pitch_setpoint_get_pitch_setpoint(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Decode a roll_pitch_setpoint message into a struct
 *
 * @param msg The message to decode
 * @param roll_pitch_setpoint C-struct to decode the message contents into
 */
static inline void mavlink_msg_roll_pitch_setpoint_decode(const mavlink_message_t* msg, mavlink_roll_pitch_setpoint_t* roll_pitch_setpoint)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    roll_pitch_setpoint->roll_setpoint = mavlink_msg_roll_pitch_setpoint_get_roll_setpoint(msg);
    roll_pitch_setpoint->pitch_setpoint = mavlink_msg_roll_pitch_setpoint_get_pitch_setpoint(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ROLL_PITCH_SETPOINT_LEN? msg->len : MAVLINK_MSG_ID_ROLL_PITCH_SETPOINT_LEN;
        memset(roll_pitch_setpoint, 0, MAVLINK_MSG_ID_ROLL_PITCH_SETPOINT_LEN);
    memcpy(roll_pitch_setpoint, _MAV_PAYLOAD(msg), len);
#endif
}
