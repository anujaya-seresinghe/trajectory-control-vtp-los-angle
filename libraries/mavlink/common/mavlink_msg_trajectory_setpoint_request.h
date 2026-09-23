#pragma once
// MESSAGE TRAJECTORY_SETPOINT_REQUEST PACKING

#define MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_REQUEST 600


typedef struct __mavlink_trajectory_setpoint_request_t {
 uint16_t index; /*<  index of the waypoint*/
} mavlink_trajectory_setpoint_request_t;

#define MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_REQUEST_LEN 2
#define MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_REQUEST_MIN_LEN 2
#define MAVLINK_MSG_ID_600_LEN 2
#define MAVLINK_MSG_ID_600_MIN_LEN 2

#define MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_REQUEST_CRC 175
#define MAVLINK_MSG_ID_600_CRC 175



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_TRAJECTORY_SETPOINT_REQUEST { \
    600, \
    "TRAJECTORY_SETPOINT_REQUEST", \
    1, \
    {  { "index", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_trajectory_setpoint_request_t, index) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_TRAJECTORY_SETPOINT_REQUEST { \
    "TRAJECTORY_SETPOINT_REQUEST", \
    1, \
    {  { "index", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_trajectory_setpoint_request_t, index) }, \
         } \
}
#endif

/**
 * @brief Pack a trajectory_setpoint_request message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param index  index of the waypoint
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_trajectory_setpoint_request_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint16_t index)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_REQUEST_LEN];
    _mav_put_uint16_t(buf, 0, index);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_REQUEST_LEN);
#else
    mavlink_trajectory_setpoint_request_t packet;
    packet.index = index;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_REQUEST_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_REQUEST;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_REQUEST_MIN_LEN, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_REQUEST_LEN, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_REQUEST_CRC);
}

/**
 * @brief Pack a trajectory_setpoint_request message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param index  index of the waypoint
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_trajectory_setpoint_request_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint16_t index)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_REQUEST_LEN];
    _mav_put_uint16_t(buf, 0, index);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_REQUEST_LEN);
#else
    mavlink_trajectory_setpoint_request_t packet;
    packet.index = index;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_REQUEST_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_REQUEST;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_REQUEST_MIN_LEN, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_REQUEST_LEN, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_REQUEST_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_REQUEST_MIN_LEN, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_REQUEST_LEN);
#endif
}

/**
 * @brief Pack a trajectory_setpoint_request message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param index  index of the waypoint
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_trajectory_setpoint_request_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint16_t index)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_REQUEST_LEN];
    _mav_put_uint16_t(buf, 0, index);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_REQUEST_LEN);
#else
    mavlink_trajectory_setpoint_request_t packet;
    packet.index = index;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_REQUEST_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_REQUEST;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_REQUEST_MIN_LEN, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_REQUEST_LEN, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_REQUEST_CRC);
}

/**
 * @brief Encode a trajectory_setpoint_request struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param trajectory_setpoint_request C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_trajectory_setpoint_request_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_trajectory_setpoint_request_t* trajectory_setpoint_request)
{
    return mavlink_msg_trajectory_setpoint_request_pack(system_id, component_id, msg, trajectory_setpoint_request->index);
}

/**
 * @brief Encode a trajectory_setpoint_request struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param trajectory_setpoint_request C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_trajectory_setpoint_request_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_trajectory_setpoint_request_t* trajectory_setpoint_request)
{
    return mavlink_msg_trajectory_setpoint_request_pack_chan(system_id, component_id, chan, msg, trajectory_setpoint_request->index);
}

/**
 * @brief Encode a trajectory_setpoint_request struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param trajectory_setpoint_request C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_trajectory_setpoint_request_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_trajectory_setpoint_request_t* trajectory_setpoint_request)
{
    return mavlink_msg_trajectory_setpoint_request_pack_status(system_id, component_id, _status, msg,  trajectory_setpoint_request->index);
}

/**
 * @brief Send a trajectory_setpoint_request message
 * @param chan MAVLink channel to send the message
 *
 * @param index  index of the waypoint
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_trajectory_setpoint_request_send(mavlink_channel_t chan, uint16_t index)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_REQUEST_LEN];
    _mav_put_uint16_t(buf, 0, index);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_REQUEST, buf, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_REQUEST_MIN_LEN, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_REQUEST_LEN, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_REQUEST_CRC);
#else
    mavlink_trajectory_setpoint_request_t packet;
    packet.index = index;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_REQUEST, (const char *)&packet, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_REQUEST_MIN_LEN, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_REQUEST_LEN, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_REQUEST_CRC);
#endif
}

/**
 * @brief Send a trajectory_setpoint_request message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_trajectory_setpoint_request_send_struct(mavlink_channel_t chan, const mavlink_trajectory_setpoint_request_t* trajectory_setpoint_request)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_trajectory_setpoint_request_send(chan, trajectory_setpoint_request->index);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_REQUEST, (const char *)trajectory_setpoint_request, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_REQUEST_MIN_LEN, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_REQUEST_LEN, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_REQUEST_CRC);
#endif
}

#if MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_REQUEST_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_trajectory_setpoint_request_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint16_t index)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint16_t(buf, 0, index);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_REQUEST, buf, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_REQUEST_MIN_LEN, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_REQUEST_LEN, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_REQUEST_CRC);
#else
    mavlink_trajectory_setpoint_request_t *packet = (mavlink_trajectory_setpoint_request_t *)msgbuf;
    packet->index = index;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_REQUEST, (const char *)packet, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_REQUEST_MIN_LEN, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_REQUEST_LEN, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_REQUEST_CRC);
#endif
}
#endif

#endif

// MESSAGE TRAJECTORY_SETPOINT_REQUEST UNPACKING


/**
 * @brief Get field index from trajectory_setpoint_request message
 *
 * @return  index of the waypoint
 */
static inline uint16_t mavlink_msg_trajectory_setpoint_request_get_index(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Decode a trajectory_setpoint_request message into a struct
 *
 * @param msg The message to decode
 * @param trajectory_setpoint_request C-struct to decode the message contents into
 */
static inline void mavlink_msg_trajectory_setpoint_request_decode(const mavlink_message_t* msg, mavlink_trajectory_setpoint_request_t* trajectory_setpoint_request)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    trajectory_setpoint_request->index = mavlink_msg_trajectory_setpoint_request_get_index(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_REQUEST_LEN? msg->len : MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_REQUEST_LEN;
        memset(trajectory_setpoint_request, 0, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_REQUEST_LEN);
    memcpy(trajectory_setpoint_request, _MAV_PAYLOAD(msg), len);
#endif
}
