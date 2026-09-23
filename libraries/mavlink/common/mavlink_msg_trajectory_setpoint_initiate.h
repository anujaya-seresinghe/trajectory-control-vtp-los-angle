#pragma once
// MESSAGE TRAJECTORY_SETPOINT_INITIATE PACKING

#define MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_INITIATE 602


typedef struct __mavlink_trajectory_setpoint_initiate_t {
 uint16_t no_of_waypoints; /*<  nomber */
 uint8_t id; /*<  id of the set of waypoints*/
} mavlink_trajectory_setpoint_initiate_t;

#define MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_INITIATE_LEN 3
#define MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_INITIATE_MIN_LEN 3
#define MAVLINK_MSG_ID_602_LEN 3
#define MAVLINK_MSG_ID_602_MIN_LEN 3

#define MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_INITIATE_CRC 93
#define MAVLINK_MSG_ID_602_CRC 93



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_TRAJECTORY_SETPOINT_INITIATE { \
    602, \
    "TRAJECTORY_SETPOINT_INITIATE", \
    2, \
    {  { "no_of_waypoints", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_trajectory_setpoint_initiate_t, no_of_waypoints) }, \
         { "id", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_trajectory_setpoint_initiate_t, id) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_TRAJECTORY_SETPOINT_INITIATE { \
    "TRAJECTORY_SETPOINT_INITIATE", \
    2, \
    {  { "no_of_waypoints", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_trajectory_setpoint_initiate_t, no_of_waypoints) }, \
         { "id", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_trajectory_setpoint_initiate_t, id) }, \
         } \
}
#endif

/**
 * @brief Pack a trajectory_setpoint_initiate message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param no_of_waypoints  nomber 
 * @param id  id of the set of waypoints
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_trajectory_setpoint_initiate_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint16_t no_of_waypoints, uint8_t id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_INITIATE_LEN];
    _mav_put_uint16_t(buf, 0, no_of_waypoints);
    _mav_put_uint8_t(buf, 2, id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_INITIATE_LEN);
#else
    mavlink_trajectory_setpoint_initiate_t packet;
    packet.no_of_waypoints = no_of_waypoints;
    packet.id = id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_INITIATE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_INITIATE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_INITIATE_MIN_LEN, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_INITIATE_LEN, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_INITIATE_CRC);
}

/**
 * @brief Pack a trajectory_setpoint_initiate message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param no_of_waypoints  nomber 
 * @param id  id of the set of waypoints
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_trajectory_setpoint_initiate_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint16_t no_of_waypoints, uint8_t id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_INITIATE_LEN];
    _mav_put_uint16_t(buf, 0, no_of_waypoints);
    _mav_put_uint8_t(buf, 2, id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_INITIATE_LEN);
#else
    mavlink_trajectory_setpoint_initiate_t packet;
    packet.no_of_waypoints = no_of_waypoints;
    packet.id = id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_INITIATE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_INITIATE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_INITIATE_MIN_LEN, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_INITIATE_LEN, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_INITIATE_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_INITIATE_MIN_LEN, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_INITIATE_LEN);
#endif
}

/**
 * @brief Pack a trajectory_setpoint_initiate message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param no_of_waypoints  nomber 
 * @param id  id of the set of waypoints
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_trajectory_setpoint_initiate_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint16_t no_of_waypoints,uint8_t id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_INITIATE_LEN];
    _mav_put_uint16_t(buf, 0, no_of_waypoints);
    _mav_put_uint8_t(buf, 2, id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_INITIATE_LEN);
#else
    mavlink_trajectory_setpoint_initiate_t packet;
    packet.no_of_waypoints = no_of_waypoints;
    packet.id = id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_INITIATE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_INITIATE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_INITIATE_MIN_LEN, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_INITIATE_LEN, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_INITIATE_CRC);
}

/**
 * @brief Encode a trajectory_setpoint_initiate struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param trajectory_setpoint_initiate C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_trajectory_setpoint_initiate_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_trajectory_setpoint_initiate_t* trajectory_setpoint_initiate)
{
    return mavlink_msg_trajectory_setpoint_initiate_pack(system_id, component_id, msg, trajectory_setpoint_initiate->no_of_waypoints, trajectory_setpoint_initiate->id);
}

/**
 * @brief Encode a trajectory_setpoint_initiate struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param trajectory_setpoint_initiate C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_trajectory_setpoint_initiate_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_trajectory_setpoint_initiate_t* trajectory_setpoint_initiate)
{
    return mavlink_msg_trajectory_setpoint_initiate_pack_chan(system_id, component_id, chan, msg, trajectory_setpoint_initiate->no_of_waypoints, trajectory_setpoint_initiate->id);
}

/**
 * @brief Encode a trajectory_setpoint_initiate struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param trajectory_setpoint_initiate C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_trajectory_setpoint_initiate_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_trajectory_setpoint_initiate_t* trajectory_setpoint_initiate)
{
    return mavlink_msg_trajectory_setpoint_initiate_pack_status(system_id, component_id, _status, msg,  trajectory_setpoint_initiate->no_of_waypoints, trajectory_setpoint_initiate->id);
}

/**
 * @brief Send a trajectory_setpoint_initiate message
 * @param chan MAVLink channel to send the message
 *
 * @param no_of_waypoints  nomber 
 * @param id  id of the set of waypoints
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_trajectory_setpoint_initiate_send(mavlink_channel_t chan, uint16_t no_of_waypoints, uint8_t id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_INITIATE_LEN];
    _mav_put_uint16_t(buf, 0, no_of_waypoints);
    _mav_put_uint8_t(buf, 2, id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_INITIATE, buf, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_INITIATE_MIN_LEN, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_INITIATE_LEN, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_INITIATE_CRC);
#else
    mavlink_trajectory_setpoint_initiate_t packet;
    packet.no_of_waypoints = no_of_waypoints;
    packet.id = id;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_INITIATE, (const char *)&packet, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_INITIATE_MIN_LEN, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_INITIATE_LEN, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_INITIATE_CRC);
#endif
}

/**
 * @brief Send a trajectory_setpoint_initiate message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_trajectory_setpoint_initiate_send_struct(mavlink_channel_t chan, const mavlink_trajectory_setpoint_initiate_t* trajectory_setpoint_initiate)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_trajectory_setpoint_initiate_send(chan, trajectory_setpoint_initiate->no_of_waypoints, trajectory_setpoint_initiate->id);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_INITIATE, (const char *)trajectory_setpoint_initiate, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_INITIATE_MIN_LEN, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_INITIATE_LEN, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_INITIATE_CRC);
#endif
}

#if MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_INITIATE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_trajectory_setpoint_initiate_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint16_t no_of_waypoints, uint8_t id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint16_t(buf, 0, no_of_waypoints);
    _mav_put_uint8_t(buf, 2, id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_INITIATE, buf, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_INITIATE_MIN_LEN, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_INITIATE_LEN, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_INITIATE_CRC);
#else
    mavlink_trajectory_setpoint_initiate_t *packet = (mavlink_trajectory_setpoint_initiate_t *)msgbuf;
    packet->no_of_waypoints = no_of_waypoints;
    packet->id = id;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_INITIATE, (const char *)packet, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_INITIATE_MIN_LEN, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_INITIATE_LEN, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_INITIATE_CRC);
#endif
}
#endif

#endif

// MESSAGE TRAJECTORY_SETPOINT_INITIATE UNPACKING


/**
 * @brief Get field no_of_waypoints from trajectory_setpoint_initiate message
 *
 * @return  nomber 
 */
static inline uint16_t mavlink_msg_trajectory_setpoint_initiate_get_no_of_waypoints(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field id from trajectory_setpoint_initiate message
 *
 * @return  id of the set of waypoints
 */
static inline uint8_t mavlink_msg_trajectory_setpoint_initiate_get_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Decode a trajectory_setpoint_initiate message into a struct
 *
 * @param msg The message to decode
 * @param trajectory_setpoint_initiate C-struct to decode the message contents into
 */
static inline void mavlink_msg_trajectory_setpoint_initiate_decode(const mavlink_message_t* msg, mavlink_trajectory_setpoint_initiate_t* trajectory_setpoint_initiate)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    trajectory_setpoint_initiate->no_of_waypoints = mavlink_msg_trajectory_setpoint_initiate_get_no_of_waypoints(msg);
    trajectory_setpoint_initiate->id = mavlink_msg_trajectory_setpoint_initiate_get_id(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_INITIATE_LEN? msg->len : MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_INITIATE_LEN;
        memset(trajectory_setpoint_initiate, 0, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_INITIATE_LEN);
    memcpy(trajectory_setpoint_initiate, _MAV_PAYLOAD(msg), len);
#endif
}
