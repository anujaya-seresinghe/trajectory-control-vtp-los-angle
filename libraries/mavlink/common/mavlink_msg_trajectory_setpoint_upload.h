#pragma once
// MESSAGE TRAJECTORY_SETPOINT_UPLOAD PACKING

#define MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_UPLOAD 601


typedef struct __mavlink_trajectory_setpoint_upload_t {
 float x; /*<  x coordinate in NED frame*/
 float y; /*<  y coordinate*/
 float vx; /*<  velocity in x direction*/
 float vy; /*<  velocity in y direction*/
 float at; /*<  lateral acceleration*/
 float jt; /*<  jerk */
 float t; /*<  timestamp of the waypoint */
 uint16_t index; /*<  index of the waypoint*/
 uint8_t id; /*<  id of the set of waypoints*/
} mavlink_trajectory_setpoint_upload_t;

#define MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_UPLOAD_LEN 31
#define MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_UPLOAD_MIN_LEN 31
#define MAVLINK_MSG_ID_601_LEN 31
#define MAVLINK_MSG_ID_601_MIN_LEN 31

#define MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_UPLOAD_CRC 41
#define MAVLINK_MSG_ID_601_CRC 41



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_TRAJECTORY_SETPOINT_UPLOAD { \
    601, \
    "TRAJECTORY_SETPOINT_UPLOAD", \
    9, \
    {  { "index", NULL, MAVLINK_TYPE_UINT16_T, 0, 28, offsetof(mavlink_trajectory_setpoint_upload_t, index) }, \
         { "id", NULL, MAVLINK_TYPE_UINT8_T, 0, 30, offsetof(mavlink_trajectory_setpoint_upload_t, id) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_trajectory_setpoint_upload_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_trajectory_setpoint_upload_t, y) }, \
         { "vx", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_trajectory_setpoint_upload_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_trajectory_setpoint_upload_t, vy) }, \
         { "at", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_trajectory_setpoint_upload_t, at) }, \
         { "jt", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_trajectory_setpoint_upload_t, jt) }, \
         { "t", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_trajectory_setpoint_upload_t, t) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_TRAJECTORY_SETPOINT_UPLOAD { \
    "TRAJECTORY_SETPOINT_UPLOAD", \
    9, \
    {  { "index", NULL, MAVLINK_TYPE_UINT16_T, 0, 28, offsetof(mavlink_trajectory_setpoint_upload_t, index) }, \
         { "id", NULL, MAVLINK_TYPE_UINT8_T, 0, 30, offsetof(mavlink_trajectory_setpoint_upload_t, id) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_trajectory_setpoint_upload_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_trajectory_setpoint_upload_t, y) }, \
         { "vx", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_trajectory_setpoint_upload_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_trajectory_setpoint_upload_t, vy) }, \
         { "at", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_trajectory_setpoint_upload_t, at) }, \
         { "jt", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_trajectory_setpoint_upload_t, jt) }, \
         { "t", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_trajectory_setpoint_upload_t, t) }, \
         } \
}
#endif

/**
 * @brief Pack a trajectory_setpoint_upload message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param index  index of the waypoint
 * @param id  id of the set of waypoints
 * @param x  x coordinate in NED frame
 * @param y  y coordinate
 * @param vx  velocity in x direction
 * @param vy  velocity in y direction
 * @param at  lateral acceleration
 * @param jt  jerk 
 * @param t  timestamp of the waypoint 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_trajectory_setpoint_upload_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint16_t index, uint8_t id, float x, float y, float vx, float vy, float at, float jt, float t)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_UPLOAD_LEN];
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, vx);
    _mav_put_float(buf, 12, vy);
    _mav_put_float(buf, 16, at);
    _mav_put_float(buf, 20, jt);
    _mav_put_float(buf, 24, t);
    _mav_put_uint16_t(buf, 28, index);
    _mav_put_uint8_t(buf, 30, id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_UPLOAD_LEN);
#else
    mavlink_trajectory_setpoint_upload_t packet;
    packet.x = x;
    packet.y = y;
    packet.vx = vx;
    packet.vy = vy;
    packet.at = at;
    packet.jt = jt;
    packet.t = t;
    packet.index = index;
    packet.id = id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_UPLOAD_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_UPLOAD;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_UPLOAD_MIN_LEN, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_UPLOAD_LEN, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_UPLOAD_CRC);
}

/**
 * @brief Pack a trajectory_setpoint_upload message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param index  index of the waypoint
 * @param id  id of the set of waypoints
 * @param x  x coordinate in NED frame
 * @param y  y coordinate
 * @param vx  velocity in x direction
 * @param vy  velocity in y direction
 * @param at  lateral acceleration
 * @param jt  jerk 
 * @param t  timestamp of the waypoint 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_trajectory_setpoint_upload_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint16_t index, uint8_t id, float x, float y, float vx, float vy, float at, float jt, float t)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_UPLOAD_LEN];
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, vx);
    _mav_put_float(buf, 12, vy);
    _mav_put_float(buf, 16, at);
    _mav_put_float(buf, 20, jt);
    _mav_put_float(buf, 24, t);
    _mav_put_uint16_t(buf, 28, index);
    _mav_put_uint8_t(buf, 30, id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_UPLOAD_LEN);
#else
    mavlink_trajectory_setpoint_upload_t packet;
    packet.x = x;
    packet.y = y;
    packet.vx = vx;
    packet.vy = vy;
    packet.at = at;
    packet.jt = jt;
    packet.t = t;
    packet.index = index;
    packet.id = id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_UPLOAD_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_UPLOAD;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_UPLOAD_MIN_LEN, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_UPLOAD_LEN, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_UPLOAD_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_UPLOAD_MIN_LEN, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_UPLOAD_LEN);
#endif
}

/**
 * @brief Pack a trajectory_setpoint_upload message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param index  index of the waypoint
 * @param id  id of the set of waypoints
 * @param x  x coordinate in NED frame
 * @param y  y coordinate
 * @param vx  velocity in x direction
 * @param vy  velocity in y direction
 * @param at  lateral acceleration
 * @param jt  jerk 
 * @param t  timestamp of the waypoint 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_trajectory_setpoint_upload_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint16_t index,uint8_t id,float x,float y,float vx,float vy,float at,float jt,float t)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_UPLOAD_LEN];
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, vx);
    _mav_put_float(buf, 12, vy);
    _mav_put_float(buf, 16, at);
    _mav_put_float(buf, 20, jt);
    _mav_put_float(buf, 24, t);
    _mav_put_uint16_t(buf, 28, index);
    _mav_put_uint8_t(buf, 30, id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_UPLOAD_LEN);
#else
    mavlink_trajectory_setpoint_upload_t packet;
    packet.x = x;
    packet.y = y;
    packet.vx = vx;
    packet.vy = vy;
    packet.at = at;
    packet.jt = jt;
    packet.t = t;
    packet.index = index;
    packet.id = id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_UPLOAD_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_UPLOAD;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_UPLOAD_MIN_LEN, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_UPLOAD_LEN, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_UPLOAD_CRC);
}

/**
 * @brief Encode a trajectory_setpoint_upload struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param trajectory_setpoint_upload C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_trajectory_setpoint_upload_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_trajectory_setpoint_upload_t* trajectory_setpoint_upload)
{
    return mavlink_msg_trajectory_setpoint_upload_pack(system_id, component_id, msg, trajectory_setpoint_upload->index, trajectory_setpoint_upload->id, trajectory_setpoint_upload->x, trajectory_setpoint_upload->y, trajectory_setpoint_upload->vx, trajectory_setpoint_upload->vy, trajectory_setpoint_upload->at, trajectory_setpoint_upload->jt, trajectory_setpoint_upload->t);
}

/**
 * @brief Encode a trajectory_setpoint_upload struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param trajectory_setpoint_upload C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_trajectory_setpoint_upload_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_trajectory_setpoint_upload_t* trajectory_setpoint_upload)
{
    return mavlink_msg_trajectory_setpoint_upload_pack_chan(system_id, component_id, chan, msg, trajectory_setpoint_upload->index, trajectory_setpoint_upload->id, trajectory_setpoint_upload->x, trajectory_setpoint_upload->y, trajectory_setpoint_upload->vx, trajectory_setpoint_upload->vy, trajectory_setpoint_upload->at, trajectory_setpoint_upload->jt, trajectory_setpoint_upload->t);
}

/**
 * @brief Encode a trajectory_setpoint_upload struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param trajectory_setpoint_upload C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_trajectory_setpoint_upload_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_trajectory_setpoint_upload_t* trajectory_setpoint_upload)
{
    return mavlink_msg_trajectory_setpoint_upload_pack_status(system_id, component_id, _status, msg,  trajectory_setpoint_upload->index, trajectory_setpoint_upload->id, trajectory_setpoint_upload->x, trajectory_setpoint_upload->y, trajectory_setpoint_upload->vx, trajectory_setpoint_upload->vy, trajectory_setpoint_upload->at, trajectory_setpoint_upload->jt, trajectory_setpoint_upload->t);
}

/**
 * @brief Send a trajectory_setpoint_upload message
 * @param chan MAVLink channel to send the message
 *
 * @param index  index of the waypoint
 * @param id  id of the set of waypoints
 * @param x  x coordinate in NED frame
 * @param y  y coordinate
 * @param vx  velocity in x direction
 * @param vy  velocity in y direction
 * @param at  lateral acceleration
 * @param jt  jerk 
 * @param t  timestamp of the waypoint 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_trajectory_setpoint_upload_send(mavlink_channel_t chan, uint16_t index, uint8_t id, float x, float y, float vx, float vy, float at, float jt, float t)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_UPLOAD_LEN];
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, vx);
    _mav_put_float(buf, 12, vy);
    _mav_put_float(buf, 16, at);
    _mav_put_float(buf, 20, jt);
    _mav_put_float(buf, 24, t);
    _mav_put_uint16_t(buf, 28, index);
    _mav_put_uint8_t(buf, 30, id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_UPLOAD, buf, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_UPLOAD_MIN_LEN, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_UPLOAD_LEN, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_UPLOAD_CRC);
#else
    mavlink_trajectory_setpoint_upload_t packet;
    packet.x = x;
    packet.y = y;
    packet.vx = vx;
    packet.vy = vy;
    packet.at = at;
    packet.jt = jt;
    packet.t = t;
    packet.index = index;
    packet.id = id;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_UPLOAD, (const char *)&packet, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_UPLOAD_MIN_LEN, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_UPLOAD_LEN, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_UPLOAD_CRC);
#endif
}

/**
 * @brief Send a trajectory_setpoint_upload message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_trajectory_setpoint_upload_send_struct(mavlink_channel_t chan, const mavlink_trajectory_setpoint_upload_t* trajectory_setpoint_upload)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_trajectory_setpoint_upload_send(chan, trajectory_setpoint_upload->index, trajectory_setpoint_upload->id, trajectory_setpoint_upload->x, trajectory_setpoint_upload->y, trajectory_setpoint_upload->vx, trajectory_setpoint_upload->vy, trajectory_setpoint_upload->at, trajectory_setpoint_upload->jt, trajectory_setpoint_upload->t);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_UPLOAD, (const char *)trajectory_setpoint_upload, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_UPLOAD_MIN_LEN, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_UPLOAD_LEN, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_UPLOAD_CRC);
#endif
}

#if MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_UPLOAD_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_trajectory_setpoint_upload_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint16_t index, uint8_t id, float x, float y, float vx, float vy, float at, float jt, float t)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, vx);
    _mav_put_float(buf, 12, vy);
    _mav_put_float(buf, 16, at);
    _mav_put_float(buf, 20, jt);
    _mav_put_float(buf, 24, t);
    _mav_put_uint16_t(buf, 28, index);
    _mav_put_uint8_t(buf, 30, id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_UPLOAD, buf, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_UPLOAD_MIN_LEN, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_UPLOAD_LEN, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_UPLOAD_CRC);
#else
    mavlink_trajectory_setpoint_upload_t *packet = (mavlink_trajectory_setpoint_upload_t *)msgbuf;
    packet->x = x;
    packet->y = y;
    packet->vx = vx;
    packet->vy = vy;
    packet->at = at;
    packet->jt = jt;
    packet->t = t;
    packet->index = index;
    packet->id = id;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_UPLOAD, (const char *)packet, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_UPLOAD_MIN_LEN, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_UPLOAD_LEN, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_UPLOAD_CRC);
#endif
}
#endif

#endif

// MESSAGE TRAJECTORY_SETPOINT_UPLOAD UNPACKING


/**
 * @brief Get field index from trajectory_setpoint_upload message
 *
 * @return  index of the waypoint
 */
static inline uint16_t mavlink_msg_trajectory_setpoint_upload_get_index(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  28);
}

/**
 * @brief Get field id from trajectory_setpoint_upload message
 *
 * @return  id of the set of waypoints
 */
static inline uint8_t mavlink_msg_trajectory_setpoint_upload_get_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  30);
}

/**
 * @brief Get field x from trajectory_setpoint_upload message
 *
 * @return  x coordinate in NED frame
 */
static inline float mavlink_msg_trajectory_setpoint_upload_get_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field y from trajectory_setpoint_upload message
 *
 * @return  y coordinate
 */
static inline float mavlink_msg_trajectory_setpoint_upload_get_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field vx from trajectory_setpoint_upload message
 *
 * @return  velocity in x direction
 */
static inline float mavlink_msg_trajectory_setpoint_upload_get_vx(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field vy from trajectory_setpoint_upload message
 *
 * @return  velocity in y direction
 */
static inline float mavlink_msg_trajectory_setpoint_upload_get_vy(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field at from trajectory_setpoint_upload message
 *
 * @return  lateral acceleration
 */
static inline float mavlink_msg_trajectory_setpoint_upload_get_at(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field jt from trajectory_setpoint_upload message
 *
 * @return  jerk 
 */
static inline float mavlink_msg_trajectory_setpoint_upload_get_jt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field t from trajectory_setpoint_upload message
 *
 * @return  timestamp of the waypoint 
 */
static inline float mavlink_msg_trajectory_setpoint_upload_get_t(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Decode a trajectory_setpoint_upload message into a struct
 *
 * @param msg The message to decode
 * @param trajectory_setpoint_upload C-struct to decode the message contents into
 */
static inline void mavlink_msg_trajectory_setpoint_upload_decode(const mavlink_message_t* msg, mavlink_trajectory_setpoint_upload_t* trajectory_setpoint_upload)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    trajectory_setpoint_upload->x = mavlink_msg_trajectory_setpoint_upload_get_x(msg);
    trajectory_setpoint_upload->y = mavlink_msg_trajectory_setpoint_upload_get_y(msg);
    trajectory_setpoint_upload->vx = mavlink_msg_trajectory_setpoint_upload_get_vx(msg);
    trajectory_setpoint_upload->vy = mavlink_msg_trajectory_setpoint_upload_get_vy(msg);
    trajectory_setpoint_upload->at = mavlink_msg_trajectory_setpoint_upload_get_at(msg);
    trajectory_setpoint_upload->jt = mavlink_msg_trajectory_setpoint_upload_get_jt(msg);
    trajectory_setpoint_upload->t = mavlink_msg_trajectory_setpoint_upload_get_t(msg);
    trajectory_setpoint_upload->index = mavlink_msg_trajectory_setpoint_upload_get_index(msg);
    trajectory_setpoint_upload->id = mavlink_msg_trajectory_setpoint_upload_get_id(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_UPLOAD_LEN? msg->len : MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_UPLOAD_LEN;
        memset(trajectory_setpoint_upload, 0, MAVLINK_MSG_ID_TRAJECTORY_SETPOINT_UPLOAD_LEN);
    memcpy(trajectory_setpoint_upload, _MAV_PAYLOAD(msg), len);
#endif
}
