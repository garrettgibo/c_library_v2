#pragma once
// MESSAGE NEW_XY_STATUS PACKING

#define MAVLINK_MSG_ID_NEW_XY_STATUS 181


typedef struct __mavlink_new_xy_status_t {
 float new_x; /*<  Value for new x*/
 float new_y; /*<  Value for new y*/
} mavlink_new_xy_status_t;

#define MAVLINK_MSG_ID_NEW_XY_STATUS_LEN 8
#define MAVLINK_MSG_ID_NEW_XY_STATUS_MIN_LEN 8
#define MAVLINK_MSG_ID_181_LEN 8
#define MAVLINK_MSG_ID_181_MIN_LEN 8

#define MAVLINK_MSG_ID_NEW_XY_STATUS_CRC 82
#define MAVLINK_MSG_ID_181_CRC 82



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_NEW_XY_STATUS { \
    181, \
    "NEW_XY_STATUS", \
    2, \
    {  { "new_x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_new_xy_status_t, new_x) }, \
         { "new_y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_new_xy_status_t, new_y) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_NEW_XY_STATUS { \
    "NEW_XY_STATUS", \
    2, \
    {  { "new_x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_new_xy_status_t, new_x) }, \
         { "new_y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_new_xy_status_t, new_y) }, \
         } \
}
#endif

/**
 * @brief Pack a new_xy_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param new_x  Value for new x
 * @param new_y  Value for new y
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_new_xy_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float new_x, float new_y)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_NEW_XY_STATUS_LEN];
    _mav_put_float(buf, 0, new_x);
    _mav_put_float(buf, 4, new_y);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_NEW_XY_STATUS_LEN);
#else
    mavlink_new_xy_status_t packet;
    packet.new_x = new_x;
    packet.new_y = new_y;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_NEW_XY_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_NEW_XY_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_NEW_XY_STATUS_MIN_LEN, MAVLINK_MSG_ID_NEW_XY_STATUS_LEN, MAVLINK_MSG_ID_NEW_XY_STATUS_CRC);
}

/**
 * @brief Pack a new_xy_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param new_x  Value for new x
 * @param new_y  Value for new y
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_new_xy_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float new_x,float new_y)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_NEW_XY_STATUS_LEN];
    _mav_put_float(buf, 0, new_x);
    _mav_put_float(buf, 4, new_y);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_NEW_XY_STATUS_LEN);
#else
    mavlink_new_xy_status_t packet;
    packet.new_x = new_x;
    packet.new_y = new_y;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_NEW_XY_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_NEW_XY_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_NEW_XY_STATUS_MIN_LEN, MAVLINK_MSG_ID_NEW_XY_STATUS_LEN, MAVLINK_MSG_ID_NEW_XY_STATUS_CRC);
}

/**
 * @brief Encode a new_xy_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param new_xy_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_new_xy_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_new_xy_status_t* new_xy_status)
{
    return mavlink_msg_new_xy_status_pack(system_id, component_id, msg, new_xy_status->new_x, new_xy_status->new_y);
}

/**
 * @brief Encode a new_xy_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param new_xy_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_new_xy_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_new_xy_status_t* new_xy_status)
{
    return mavlink_msg_new_xy_status_pack_chan(system_id, component_id, chan, msg, new_xy_status->new_x, new_xy_status->new_y);
}

/**
 * @brief Send a new_xy_status message
 * @param chan MAVLink channel to send the message
 *
 * @param new_x  Value for new x
 * @param new_y  Value for new y
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_new_xy_status_send(mavlink_channel_t chan, float new_x, float new_y)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_NEW_XY_STATUS_LEN];
    _mav_put_float(buf, 0, new_x);
    _mav_put_float(buf, 4, new_y);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NEW_XY_STATUS, buf, MAVLINK_MSG_ID_NEW_XY_STATUS_MIN_LEN, MAVLINK_MSG_ID_NEW_XY_STATUS_LEN, MAVLINK_MSG_ID_NEW_XY_STATUS_CRC);
#else
    mavlink_new_xy_status_t packet;
    packet.new_x = new_x;
    packet.new_y = new_y;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NEW_XY_STATUS, (const char *)&packet, MAVLINK_MSG_ID_NEW_XY_STATUS_MIN_LEN, MAVLINK_MSG_ID_NEW_XY_STATUS_LEN, MAVLINK_MSG_ID_NEW_XY_STATUS_CRC);
#endif
}

/**
 * @brief Send a new_xy_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_new_xy_status_send_struct(mavlink_channel_t chan, const mavlink_new_xy_status_t* new_xy_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_new_xy_status_send(chan, new_xy_status->new_x, new_xy_status->new_y);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NEW_XY_STATUS, (const char *)new_xy_status, MAVLINK_MSG_ID_NEW_XY_STATUS_MIN_LEN, MAVLINK_MSG_ID_NEW_XY_STATUS_LEN, MAVLINK_MSG_ID_NEW_XY_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_NEW_XY_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_new_xy_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float new_x, float new_y)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, new_x);
    _mav_put_float(buf, 4, new_y);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NEW_XY_STATUS, buf, MAVLINK_MSG_ID_NEW_XY_STATUS_MIN_LEN, MAVLINK_MSG_ID_NEW_XY_STATUS_LEN, MAVLINK_MSG_ID_NEW_XY_STATUS_CRC);
#else
    mavlink_new_xy_status_t *packet = (mavlink_new_xy_status_t *)msgbuf;
    packet->new_x = new_x;
    packet->new_y = new_y;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NEW_XY_STATUS, (const char *)packet, MAVLINK_MSG_ID_NEW_XY_STATUS_MIN_LEN, MAVLINK_MSG_ID_NEW_XY_STATUS_LEN, MAVLINK_MSG_ID_NEW_XY_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE NEW_XY_STATUS UNPACKING


/**
 * @brief Get field new_x from new_xy_status message
 *
 * @return  Value for new x
 */
static inline float mavlink_msg_new_xy_status_get_new_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field new_y from new_xy_status message
 *
 * @return  Value for new y
 */
static inline float mavlink_msg_new_xy_status_get_new_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Decode a new_xy_status message into a struct
 *
 * @param msg The message to decode
 * @param new_xy_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_new_xy_status_decode(const mavlink_message_t* msg, mavlink_new_xy_status_t* new_xy_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    new_xy_status->new_x = mavlink_msg_new_xy_status_get_new_x(msg);
    new_xy_status->new_y = mavlink_msg_new_xy_status_get_new_y(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_NEW_XY_STATUS_LEN? msg->len : MAVLINK_MSG_ID_NEW_XY_STATUS_LEN;
        memset(new_xy_status, 0, MAVLINK_MSG_ID_NEW_XY_STATUS_LEN);
    memcpy(new_xy_status, _MAV_PAYLOAD(msg), len);
#endif
}
