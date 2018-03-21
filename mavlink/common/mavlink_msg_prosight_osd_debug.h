// MESSAGE PROSIGHT_OSD_DEBUG PACKING

#define MAVLINK_MSG_ID_PROSIGHT_OSD_DEBUG 151

MAVPACKED(
typedef struct __mavlink_prosight_osd_debug_t {
 int16_t debug_1; /*< debug field #1*/
 int16_t debug_2; /*< debug field #2*/
 int16_t debug_3; /*< debug field #3*/
 int16_t debug_4; /*< debug field #4*/
}) mavlink_prosight_osd_debug_t;

#define MAVLINK_MSG_ID_PROSIGHT_OSD_DEBUG_LEN 8
#define MAVLINK_MSG_ID_PROSIGHT_OSD_DEBUG_MIN_LEN 8
#define MAVLINK_MSG_ID_151_LEN 8
#define MAVLINK_MSG_ID_151_MIN_LEN 8

#define MAVLINK_MSG_ID_PROSIGHT_OSD_DEBUG_CRC 80
#define MAVLINK_MSG_ID_151_CRC 80



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_PROSIGHT_OSD_DEBUG { \
	151, \
	"PROSIGHT_OSD_DEBUG", \
	4, \
	{  { "debug_1", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_prosight_osd_debug_t, debug_1) }, \
         { "debug_2", NULL, MAVLINK_TYPE_INT16_T, 0, 2, offsetof(mavlink_prosight_osd_debug_t, debug_2) }, \
         { "debug_3", NULL, MAVLINK_TYPE_INT16_T, 0, 4, offsetof(mavlink_prosight_osd_debug_t, debug_3) }, \
         { "debug_4", NULL, MAVLINK_TYPE_INT16_T, 0, 6, offsetof(mavlink_prosight_osd_debug_t, debug_4) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_PROSIGHT_OSD_DEBUG { \
	"PROSIGHT_OSD_DEBUG", \
	4, \
	{  { "debug_1", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_prosight_osd_debug_t, debug_1) }, \
         { "debug_2", NULL, MAVLINK_TYPE_INT16_T, 0, 2, offsetof(mavlink_prosight_osd_debug_t, debug_2) }, \
         { "debug_3", NULL, MAVLINK_TYPE_INT16_T, 0, 4, offsetof(mavlink_prosight_osd_debug_t, debug_3) }, \
         { "debug_4", NULL, MAVLINK_TYPE_INT16_T, 0, 6, offsetof(mavlink_prosight_osd_debug_t, debug_4) }, \
         } \
}
#endif

/**
 * @brief Pack a prosight_osd_debug message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param debug_1 debug field #1
 * @param debug_2 debug field #2
 * @param debug_3 debug field #3
 * @param debug_4 debug field #4
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_prosight_osd_debug_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       int16_t debug_1, int16_t debug_2, int16_t debug_3, int16_t debug_4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_PROSIGHT_OSD_DEBUG_LEN];
	_mav_put_int16_t(buf, 0, debug_1);
	_mav_put_int16_t(buf, 2, debug_2);
	_mav_put_int16_t(buf, 4, debug_3);
	_mav_put_int16_t(buf, 6, debug_4);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PROSIGHT_OSD_DEBUG_LEN);
#else
	mavlink_prosight_osd_debug_t packet;
	packet.debug_1 = debug_1;
	packet.debug_2 = debug_2;
	packet.debug_3 = debug_3;
	packet.debug_4 = debug_4;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PROSIGHT_OSD_DEBUG_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_PROSIGHT_OSD_DEBUG;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_PROSIGHT_OSD_DEBUG_MIN_LEN, MAVLINK_MSG_ID_PROSIGHT_OSD_DEBUG_LEN, MAVLINK_MSG_ID_PROSIGHT_OSD_DEBUG_CRC);
}

/**
 * @brief Pack a prosight_osd_debug message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param debug_1 debug field #1
 * @param debug_2 debug field #2
 * @param debug_3 debug field #3
 * @param debug_4 debug field #4
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_prosight_osd_debug_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           int16_t debug_1,int16_t debug_2,int16_t debug_3,int16_t debug_4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_PROSIGHT_OSD_DEBUG_LEN];
	_mav_put_int16_t(buf, 0, debug_1);
	_mav_put_int16_t(buf, 2, debug_2);
	_mav_put_int16_t(buf, 4, debug_3);
	_mav_put_int16_t(buf, 6, debug_4);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PROSIGHT_OSD_DEBUG_LEN);
#else
	mavlink_prosight_osd_debug_t packet;
	packet.debug_1 = debug_1;
	packet.debug_2 = debug_2;
	packet.debug_3 = debug_3;
	packet.debug_4 = debug_4;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PROSIGHT_OSD_DEBUG_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_PROSIGHT_OSD_DEBUG;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_PROSIGHT_OSD_DEBUG_MIN_LEN, MAVLINK_MSG_ID_PROSIGHT_OSD_DEBUG_LEN, MAVLINK_MSG_ID_PROSIGHT_OSD_DEBUG_CRC);
}

/**
 * @brief Encode a prosight_osd_debug struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param prosight_osd_debug C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_prosight_osd_debug_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_prosight_osd_debug_t* prosight_osd_debug)
{
	return mavlink_msg_prosight_osd_debug_pack(system_id, component_id, msg, prosight_osd_debug->debug_1, prosight_osd_debug->debug_2, prosight_osd_debug->debug_3, prosight_osd_debug->debug_4);
}

/**
 * @brief Encode a prosight_osd_debug struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param prosight_osd_debug C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_prosight_osd_debug_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_prosight_osd_debug_t* prosight_osd_debug)
{
	return mavlink_msg_prosight_osd_debug_pack_chan(system_id, component_id, chan, msg, prosight_osd_debug->debug_1, prosight_osd_debug->debug_2, prosight_osd_debug->debug_3, prosight_osd_debug->debug_4);
}

/**
 * @brief Send a prosight_osd_debug message
 * @param chan MAVLink channel to send the message
 *
 * @param debug_1 debug field #1
 * @param debug_2 debug field #2
 * @param debug_3 debug field #3
 * @param debug_4 debug field #4
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_prosight_osd_debug_send(mavlink_channel_t chan, int16_t debug_1, int16_t debug_2, int16_t debug_3, int16_t debug_4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_PROSIGHT_OSD_DEBUG_LEN];
	_mav_put_int16_t(buf, 0, debug_1);
	_mav_put_int16_t(buf, 2, debug_2);
	_mav_put_int16_t(buf, 4, debug_3);
	_mav_put_int16_t(buf, 6, debug_4);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PROSIGHT_OSD_DEBUG, buf, MAVLINK_MSG_ID_PROSIGHT_OSD_DEBUG_MIN_LEN, MAVLINK_MSG_ID_PROSIGHT_OSD_DEBUG_LEN, MAVLINK_MSG_ID_PROSIGHT_OSD_DEBUG_CRC);
#else
	mavlink_prosight_osd_debug_t packet;
	packet.debug_1 = debug_1;
	packet.debug_2 = debug_2;
	packet.debug_3 = debug_3;
	packet.debug_4 = debug_4;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PROSIGHT_OSD_DEBUG, (const char *)&packet, MAVLINK_MSG_ID_PROSIGHT_OSD_DEBUG_MIN_LEN, MAVLINK_MSG_ID_PROSIGHT_OSD_DEBUG_LEN, MAVLINK_MSG_ID_PROSIGHT_OSD_DEBUG_CRC);
#endif
}

/**
 * @brief Send a prosight_osd_debug message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_prosight_osd_debug_send_struct(mavlink_channel_t chan, const mavlink_prosight_osd_debug_t* prosight_osd_debug)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_prosight_osd_debug_send(chan, prosight_osd_debug->debug_1, prosight_osd_debug->debug_2, prosight_osd_debug->debug_3, prosight_osd_debug->debug_4);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PROSIGHT_OSD_DEBUG, (const char *)prosight_osd_debug, MAVLINK_MSG_ID_PROSIGHT_OSD_DEBUG_MIN_LEN, MAVLINK_MSG_ID_PROSIGHT_OSD_DEBUG_LEN, MAVLINK_MSG_ID_PROSIGHT_OSD_DEBUG_CRC);
#endif
}

#if MAVLINK_MSG_ID_PROSIGHT_OSD_DEBUG_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_prosight_osd_debug_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  int16_t debug_1, int16_t debug_2, int16_t debug_3, int16_t debug_4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_int16_t(buf, 0, debug_1);
	_mav_put_int16_t(buf, 2, debug_2);
	_mav_put_int16_t(buf, 4, debug_3);
	_mav_put_int16_t(buf, 6, debug_4);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PROSIGHT_OSD_DEBUG, buf, MAVLINK_MSG_ID_PROSIGHT_OSD_DEBUG_MIN_LEN, MAVLINK_MSG_ID_PROSIGHT_OSD_DEBUG_LEN, MAVLINK_MSG_ID_PROSIGHT_OSD_DEBUG_CRC);
#else
	mavlink_prosight_osd_debug_t *packet = (mavlink_prosight_osd_debug_t *)msgbuf;
	packet->debug_1 = debug_1;
	packet->debug_2 = debug_2;
	packet->debug_3 = debug_3;
	packet->debug_4 = debug_4;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PROSIGHT_OSD_DEBUG, (const char *)packet, MAVLINK_MSG_ID_PROSIGHT_OSD_DEBUG_MIN_LEN, MAVLINK_MSG_ID_PROSIGHT_OSD_DEBUG_LEN, MAVLINK_MSG_ID_PROSIGHT_OSD_DEBUG_CRC);
#endif
}
#endif

#endif

// MESSAGE PROSIGHT_OSD_DEBUG UNPACKING


/**
 * @brief Get field debug_1 from prosight_osd_debug message
 *
 * @return debug field #1
 */
static inline int16_t mavlink_msg_prosight_osd_debug_get_debug_1(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  0);
}

/**
 * @brief Get field debug_2 from prosight_osd_debug message
 *
 * @return debug field #2
 */
static inline int16_t mavlink_msg_prosight_osd_debug_get_debug_2(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  2);
}

/**
 * @brief Get field debug_3 from prosight_osd_debug message
 *
 * @return debug field #3
 */
static inline int16_t mavlink_msg_prosight_osd_debug_get_debug_3(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  4);
}

/**
 * @brief Get field debug_4 from prosight_osd_debug message
 *
 * @return debug field #4
 */
static inline int16_t mavlink_msg_prosight_osd_debug_get_debug_4(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  6);
}

/**
 * @brief Decode a prosight_osd_debug message into a struct
 *
 * @param msg The message to decode
 * @param prosight_osd_debug C-struct to decode the message contents into
 */
static inline void mavlink_msg_prosight_osd_debug_decode(const mavlink_message_t* msg, mavlink_prosight_osd_debug_t* prosight_osd_debug)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	prosight_osd_debug->debug_1 = mavlink_msg_prosight_osd_debug_get_debug_1(msg);
	prosight_osd_debug->debug_2 = mavlink_msg_prosight_osd_debug_get_debug_2(msg);
	prosight_osd_debug->debug_3 = mavlink_msg_prosight_osd_debug_get_debug_3(msg);
	prosight_osd_debug->debug_4 = mavlink_msg_prosight_osd_debug_get_debug_4(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_PROSIGHT_OSD_DEBUG_LEN? msg->len : MAVLINK_MSG_ID_PROSIGHT_OSD_DEBUG_LEN;
        memset(prosight_osd_debug, 0, MAVLINK_MSG_ID_PROSIGHT_OSD_DEBUG_LEN);
	memcpy(prosight_osd_debug, _MAV_PAYLOAD(msg), len);
#endif
}
