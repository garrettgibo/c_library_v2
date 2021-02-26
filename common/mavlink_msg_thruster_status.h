#pragma once
// MESSAGE THRUSTER_STATUS PACKING

#define MAVLINK_MSG_ID_THRUSTER_STATUS 183


typedef struct __mavlink_thruster_status_t {
 float thruster_1; /*<  Values for cold gas thruster 1*/
 float thruster_2; /*<  Values for cold gas thruster 2*/
 float thruster_3; /*<  Values for cold gas thruster 3*/
 float thruster_4; /*<  Values for cold gas thruster 4*/
} mavlink_thruster_status_t;

#define MAVLINK_MSG_ID_THRUSTER_STATUS_LEN 16
#define MAVLINK_MSG_ID_THRUSTER_STATUS_MIN_LEN 16
#define MAVLINK_MSG_ID_183_LEN 16
#define MAVLINK_MSG_ID_183_MIN_LEN 16

#define MAVLINK_MSG_ID_THRUSTER_STATUS_CRC 41
#define MAVLINK_MSG_ID_183_CRC 41



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_THRUSTER_STATUS { \
    183, \
    "THRUSTER_STATUS", \
    4, \
    {  { "thruster_1", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_thruster_status_t, thruster_1) }, \
         { "thruster_2", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_thruster_status_t, thruster_2) }, \
         { "thruster_3", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_thruster_status_t, thruster_3) }, \
         { "thruster_4", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_thruster_status_t, thruster_4) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_THRUSTER_STATUS { \
    "THRUSTER_STATUS", \
    4, \
    {  { "thruster_1", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_thruster_status_t, thruster_1) }, \
         { "thruster_2", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_thruster_status_t, thruster_2) }, \
         { "thruster_3", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_thruster_status_t, thruster_3) }, \
         { "thruster_4", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_thruster_status_t, thruster_4) }, \
         } \
}
#endif

/**
 * @brief Pack a thruster_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param thruster_1  Values for cold gas thruster 1
 * @param thruster_2  Values for cold gas thruster 2
 * @param thruster_3  Values for cold gas thruster 3
 * @param thruster_4  Values for cold gas thruster 4
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_thruster_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float thruster_1, float thruster_2, float thruster_3, float thruster_4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_THRUSTER_STATUS_LEN];
    _mav_put_float(buf, 0, thruster_1);
    _mav_put_float(buf, 4, thruster_2);
    _mav_put_float(buf, 8, thruster_3);
    _mav_put_float(buf, 12, thruster_4);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_THRUSTER_STATUS_LEN);
#else
    mavlink_thruster_status_t packet;
    packet.thruster_1 = thruster_1;
    packet.thruster_2 = thruster_2;
    packet.thruster_3 = thruster_3;
    packet.thruster_4 = thruster_4;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_THRUSTER_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_THRUSTER_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_THRUSTER_STATUS_MIN_LEN, MAVLINK_MSG_ID_THRUSTER_STATUS_LEN, MAVLINK_MSG_ID_THRUSTER_STATUS_CRC);
}

/**
 * @brief Pack a thruster_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param thruster_1  Values for cold gas thruster 1
 * @param thruster_2  Values for cold gas thruster 2
 * @param thruster_3  Values for cold gas thruster 3
 * @param thruster_4  Values for cold gas thruster 4
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_thruster_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float thruster_1,float thruster_2,float thruster_3,float thruster_4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_THRUSTER_STATUS_LEN];
    _mav_put_float(buf, 0, thruster_1);
    _mav_put_float(buf, 4, thruster_2);
    _mav_put_float(buf, 8, thruster_3);
    _mav_put_float(buf, 12, thruster_4);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_THRUSTER_STATUS_LEN);
#else
    mavlink_thruster_status_t packet;
    packet.thruster_1 = thruster_1;
    packet.thruster_2 = thruster_2;
    packet.thruster_3 = thruster_3;
    packet.thruster_4 = thruster_4;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_THRUSTER_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_THRUSTER_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_THRUSTER_STATUS_MIN_LEN, MAVLINK_MSG_ID_THRUSTER_STATUS_LEN, MAVLINK_MSG_ID_THRUSTER_STATUS_CRC);
}

/**
 * @brief Encode a thruster_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param thruster_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_thruster_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_thruster_status_t* thruster_status)
{
    return mavlink_msg_thruster_status_pack(system_id, component_id, msg, thruster_status->thruster_1, thruster_status->thruster_2, thruster_status->thruster_3, thruster_status->thruster_4);
}

/**
 * @brief Encode a thruster_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param thruster_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_thruster_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_thruster_status_t* thruster_status)
{
    return mavlink_msg_thruster_status_pack_chan(system_id, component_id, chan, msg, thruster_status->thruster_1, thruster_status->thruster_2, thruster_status->thruster_3, thruster_status->thruster_4);
}

/**
 * @brief Send a thruster_status message
 * @param chan MAVLink channel to send the message
 *
 * @param thruster_1  Values for cold gas thruster 1
 * @param thruster_2  Values for cold gas thruster 2
 * @param thruster_3  Values for cold gas thruster 3
 * @param thruster_4  Values for cold gas thruster 4
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_thruster_status_send(mavlink_channel_t chan, float thruster_1, float thruster_2, float thruster_3, float thruster_4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_THRUSTER_STATUS_LEN];
    _mav_put_float(buf, 0, thruster_1);
    _mav_put_float(buf, 4, thruster_2);
    _mav_put_float(buf, 8, thruster_3);
    _mav_put_float(buf, 12, thruster_4);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_THRUSTER_STATUS, buf, MAVLINK_MSG_ID_THRUSTER_STATUS_MIN_LEN, MAVLINK_MSG_ID_THRUSTER_STATUS_LEN, MAVLINK_MSG_ID_THRUSTER_STATUS_CRC);
#else
    mavlink_thruster_status_t packet;
    packet.thruster_1 = thruster_1;
    packet.thruster_2 = thruster_2;
    packet.thruster_3 = thruster_3;
    packet.thruster_4 = thruster_4;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_THRUSTER_STATUS, (const char *)&packet, MAVLINK_MSG_ID_THRUSTER_STATUS_MIN_LEN, MAVLINK_MSG_ID_THRUSTER_STATUS_LEN, MAVLINK_MSG_ID_THRUSTER_STATUS_CRC);
#endif
}

/**
 * @brief Send a thruster_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_thruster_status_send_struct(mavlink_channel_t chan, const mavlink_thruster_status_t* thruster_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_thruster_status_send(chan, thruster_status->thruster_1, thruster_status->thruster_2, thruster_status->thruster_3, thruster_status->thruster_4);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_THRUSTER_STATUS, (const char *)thruster_status, MAVLINK_MSG_ID_THRUSTER_STATUS_MIN_LEN, MAVLINK_MSG_ID_THRUSTER_STATUS_LEN, MAVLINK_MSG_ID_THRUSTER_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_THRUSTER_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_thruster_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float thruster_1, float thruster_2, float thruster_3, float thruster_4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, thruster_1);
    _mav_put_float(buf, 4, thruster_2);
    _mav_put_float(buf, 8, thruster_3);
    _mav_put_float(buf, 12, thruster_4);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_THRUSTER_STATUS, buf, MAVLINK_MSG_ID_THRUSTER_STATUS_MIN_LEN, MAVLINK_MSG_ID_THRUSTER_STATUS_LEN, MAVLINK_MSG_ID_THRUSTER_STATUS_CRC);
#else
    mavlink_thruster_status_t *packet = (mavlink_thruster_status_t *)msgbuf;
    packet->thruster_1 = thruster_1;
    packet->thruster_2 = thruster_2;
    packet->thruster_3 = thruster_3;
    packet->thruster_4 = thruster_4;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_THRUSTER_STATUS, (const char *)packet, MAVLINK_MSG_ID_THRUSTER_STATUS_MIN_LEN, MAVLINK_MSG_ID_THRUSTER_STATUS_LEN, MAVLINK_MSG_ID_THRUSTER_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE THRUSTER_STATUS UNPACKING


/**
 * @brief Get field thruster_1 from thruster_status message
 *
 * @return  Values for cold gas thruster 1
 */
static inline float mavlink_msg_thruster_status_get_thruster_1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field thruster_2 from thruster_status message
 *
 * @return  Values for cold gas thruster 2
 */
static inline float mavlink_msg_thruster_status_get_thruster_2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field thruster_3 from thruster_status message
 *
 * @return  Values for cold gas thruster 3
 */
static inline float mavlink_msg_thruster_status_get_thruster_3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field thruster_4 from thruster_status message
 *
 * @return  Values for cold gas thruster 4
 */
static inline float mavlink_msg_thruster_status_get_thruster_4(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Decode a thruster_status message into a struct
 *
 * @param msg The message to decode
 * @param thruster_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_thruster_status_decode(const mavlink_message_t* msg, mavlink_thruster_status_t* thruster_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    thruster_status->thruster_1 = mavlink_msg_thruster_status_get_thruster_1(msg);
    thruster_status->thruster_2 = mavlink_msg_thruster_status_get_thruster_2(msg);
    thruster_status->thruster_3 = mavlink_msg_thruster_status_get_thruster_3(msg);
    thruster_status->thruster_4 = mavlink_msg_thruster_status_get_thruster_4(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_THRUSTER_STATUS_LEN? msg->len : MAVLINK_MSG_ID_THRUSTER_STATUS_LEN;
        memset(thruster_status, 0, MAVLINK_MSG_ID_THRUSTER_STATUS_LEN);
    memcpy(thruster_status, _MAV_PAYLOAD(msg), len);
#endif
}
