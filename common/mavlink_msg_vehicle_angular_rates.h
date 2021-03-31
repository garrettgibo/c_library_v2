#pragma once
// MESSAGE VEHICLE_ANGULAR_RATES PACKING

#define MAVLINK_MSG_ID_VEHICLE_ANGULAR_RATES 186


typedef struct __mavlink_vehicle_angular_rates_t {
 float roll; /*<  Roll Rate*/
 float pitch; /*<  Pitch Rate*/
 float yaw; /*<  Yaw Rate*/
} mavlink_vehicle_angular_rates_t;

#define MAVLINK_MSG_ID_VEHICLE_ANGULAR_RATES_LEN 12
#define MAVLINK_MSG_ID_VEHICLE_ANGULAR_RATES_MIN_LEN 12
#define MAVLINK_MSG_ID_186_LEN 12
#define MAVLINK_MSG_ID_186_MIN_LEN 12

#define MAVLINK_MSG_ID_VEHICLE_ANGULAR_RATES_CRC 159
#define MAVLINK_MSG_ID_186_CRC 159



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_VEHICLE_ANGULAR_RATES { \
    186, \
    "VEHICLE_ANGULAR_RATES", \
    3, \
    {  { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_vehicle_angular_rates_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_vehicle_angular_rates_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_vehicle_angular_rates_t, yaw) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_VEHICLE_ANGULAR_RATES { \
    "VEHICLE_ANGULAR_RATES", \
    3, \
    {  { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_vehicle_angular_rates_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_vehicle_angular_rates_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_vehicle_angular_rates_t, yaw) }, \
         } \
}
#endif

/**
 * @brief Pack a vehicle_angular_rates message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param roll  Roll Rate
 * @param pitch  Pitch Rate
 * @param yaw  Yaw Rate
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_vehicle_angular_rates_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float roll, float pitch, float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_VEHICLE_ANGULAR_RATES_LEN];
    _mav_put_float(buf, 0, roll);
    _mav_put_float(buf, 4, pitch);
    _mav_put_float(buf, 8, yaw);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_VEHICLE_ANGULAR_RATES_LEN);
#else
    mavlink_vehicle_angular_rates_t packet;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_VEHICLE_ANGULAR_RATES_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_VEHICLE_ANGULAR_RATES;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_VEHICLE_ANGULAR_RATES_MIN_LEN, MAVLINK_MSG_ID_VEHICLE_ANGULAR_RATES_LEN, MAVLINK_MSG_ID_VEHICLE_ANGULAR_RATES_CRC);
}

/**
 * @brief Pack a vehicle_angular_rates message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param roll  Roll Rate
 * @param pitch  Pitch Rate
 * @param yaw  Yaw Rate
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_vehicle_angular_rates_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float roll,float pitch,float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_VEHICLE_ANGULAR_RATES_LEN];
    _mav_put_float(buf, 0, roll);
    _mav_put_float(buf, 4, pitch);
    _mav_put_float(buf, 8, yaw);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_VEHICLE_ANGULAR_RATES_LEN);
#else
    mavlink_vehicle_angular_rates_t packet;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_VEHICLE_ANGULAR_RATES_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_VEHICLE_ANGULAR_RATES;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_VEHICLE_ANGULAR_RATES_MIN_LEN, MAVLINK_MSG_ID_VEHICLE_ANGULAR_RATES_LEN, MAVLINK_MSG_ID_VEHICLE_ANGULAR_RATES_CRC);
}

/**
 * @brief Encode a vehicle_angular_rates struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param vehicle_angular_rates C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_vehicle_angular_rates_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_vehicle_angular_rates_t* vehicle_angular_rates)
{
    return mavlink_msg_vehicle_angular_rates_pack(system_id, component_id, msg, vehicle_angular_rates->roll, vehicle_angular_rates->pitch, vehicle_angular_rates->yaw);
}

/**
 * @brief Encode a vehicle_angular_rates struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param vehicle_angular_rates C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_vehicle_angular_rates_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_vehicle_angular_rates_t* vehicle_angular_rates)
{
    return mavlink_msg_vehicle_angular_rates_pack_chan(system_id, component_id, chan, msg, vehicle_angular_rates->roll, vehicle_angular_rates->pitch, vehicle_angular_rates->yaw);
}

/**
 * @brief Send a vehicle_angular_rates message
 * @param chan MAVLink channel to send the message
 *
 * @param roll  Roll Rate
 * @param pitch  Pitch Rate
 * @param yaw  Yaw Rate
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_vehicle_angular_rates_send(mavlink_channel_t chan, float roll, float pitch, float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_VEHICLE_ANGULAR_RATES_LEN];
    _mav_put_float(buf, 0, roll);
    _mav_put_float(buf, 4, pitch);
    _mav_put_float(buf, 8, yaw);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VEHICLE_ANGULAR_RATES, buf, MAVLINK_MSG_ID_VEHICLE_ANGULAR_RATES_MIN_LEN, MAVLINK_MSG_ID_VEHICLE_ANGULAR_RATES_LEN, MAVLINK_MSG_ID_VEHICLE_ANGULAR_RATES_CRC);
#else
    mavlink_vehicle_angular_rates_t packet;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VEHICLE_ANGULAR_RATES, (const char *)&packet, MAVLINK_MSG_ID_VEHICLE_ANGULAR_RATES_MIN_LEN, MAVLINK_MSG_ID_VEHICLE_ANGULAR_RATES_LEN, MAVLINK_MSG_ID_VEHICLE_ANGULAR_RATES_CRC);
#endif
}

/**
 * @brief Send a vehicle_angular_rates message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_vehicle_angular_rates_send_struct(mavlink_channel_t chan, const mavlink_vehicle_angular_rates_t* vehicle_angular_rates)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_vehicle_angular_rates_send(chan, vehicle_angular_rates->roll, vehicle_angular_rates->pitch, vehicle_angular_rates->yaw);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VEHICLE_ANGULAR_RATES, (const char *)vehicle_angular_rates, MAVLINK_MSG_ID_VEHICLE_ANGULAR_RATES_MIN_LEN, MAVLINK_MSG_ID_VEHICLE_ANGULAR_RATES_LEN, MAVLINK_MSG_ID_VEHICLE_ANGULAR_RATES_CRC);
#endif
}

#if MAVLINK_MSG_ID_VEHICLE_ANGULAR_RATES_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_vehicle_angular_rates_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float roll, float pitch, float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, roll);
    _mav_put_float(buf, 4, pitch);
    _mav_put_float(buf, 8, yaw);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VEHICLE_ANGULAR_RATES, buf, MAVLINK_MSG_ID_VEHICLE_ANGULAR_RATES_MIN_LEN, MAVLINK_MSG_ID_VEHICLE_ANGULAR_RATES_LEN, MAVLINK_MSG_ID_VEHICLE_ANGULAR_RATES_CRC);
#else
    mavlink_vehicle_angular_rates_t *packet = (mavlink_vehicle_angular_rates_t *)msgbuf;
    packet->roll = roll;
    packet->pitch = pitch;
    packet->yaw = yaw;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VEHICLE_ANGULAR_RATES, (const char *)packet, MAVLINK_MSG_ID_VEHICLE_ANGULAR_RATES_MIN_LEN, MAVLINK_MSG_ID_VEHICLE_ANGULAR_RATES_LEN, MAVLINK_MSG_ID_VEHICLE_ANGULAR_RATES_CRC);
#endif
}
#endif

#endif

// MESSAGE VEHICLE_ANGULAR_RATES UNPACKING


/**
 * @brief Get field roll from vehicle_angular_rates message
 *
 * @return  Roll Rate
 */
static inline float mavlink_msg_vehicle_angular_rates_get_roll(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field pitch from vehicle_angular_rates message
 *
 * @return  Pitch Rate
 */
static inline float mavlink_msg_vehicle_angular_rates_get_pitch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field yaw from vehicle_angular_rates message
 *
 * @return  Yaw Rate
 */
static inline float mavlink_msg_vehicle_angular_rates_get_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Decode a vehicle_angular_rates message into a struct
 *
 * @param msg The message to decode
 * @param vehicle_angular_rates C-struct to decode the message contents into
 */
static inline void mavlink_msg_vehicle_angular_rates_decode(const mavlink_message_t* msg, mavlink_vehicle_angular_rates_t* vehicle_angular_rates)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    vehicle_angular_rates->roll = mavlink_msg_vehicle_angular_rates_get_roll(msg);
    vehicle_angular_rates->pitch = mavlink_msg_vehicle_angular_rates_get_pitch(msg);
    vehicle_angular_rates->yaw = mavlink_msg_vehicle_angular_rates_get_yaw(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_VEHICLE_ANGULAR_RATES_LEN? msg->len : MAVLINK_MSG_ID_VEHICLE_ANGULAR_RATES_LEN;
        memset(vehicle_angular_rates, 0, MAVLINK_MSG_ID_VEHICLE_ANGULAR_RATES_LEN);
    memcpy(vehicle_angular_rates, _MAV_PAYLOAD(msg), len);
#endif
}
