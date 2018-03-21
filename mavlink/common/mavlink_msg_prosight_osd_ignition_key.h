// MESSAGE PROSIGHT_OSD_IGNITION_KEY PACKING

#define MAVLINK_MSG_ID_PROSIGHT_OSD_IGNITION_KEY 152

MAVPACKED(
typedef struct __mavlink_prosight_osd_ignition_key_t {
 uint32_t key; /*< Key challenge/answer*/
 uint8_t token; /*< Key token*/
}) mavlink_prosight_osd_ignition_key_t;

#define MAVLINK_MSG_ID_PROSIGHT_OSD_IGNITION_KEY_LEN 5
#define MAVLINK_MSG_ID_PROSIGHT_OSD_IGNITION_KEY_MIN_LEN 5
#define MAVLINK_MSG_ID_152_LEN 5
#define MAVLINK_MSG_ID_152_MIN_LEN 5

#define MAVLINK_MSG_ID_PROSIGHT_OSD_IGNITION_KEY_CRC 90
#define MAVLINK_MSG_ID_152_CRC 90



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_PROSIGHT_OSD_IGNITION_KEY { \
	152, \
	"PROSIGHT_OSD_IGNITION_KEY", \
	2, \
	{  { "key", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_prosight_osd_ignition_key_t, key) }, \
         { "token", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_prosight_osd_ignition_key_t, token) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_PROSIGHT_OSD_IGNITION_KEY { \
	"PROSIGHT_OSD_IGNITION_KEY", \
	2, \
	{  { "key", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_prosight_osd_ignition_key_t, key) }, \
         { "token", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_prosight_osd_ignition_key_t, token) }, \
         } \
}
#endif

/**
 * @brief Pack a prosight_osd_ignition_key message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param key Key challenge/answer
 * @param token Key token
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_prosight_osd_ignition_key_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint32_t key, uint8_t token)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_PROSIGHT_OSD_IGNITION_KEY_LEN];
	_mav_put_uint32_t(buf, 0, key);
	_mav_put_uint8_t(buf, 4, token);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PROSIGHT_OSD_IGNITION_KEY_LEN);
#else
	mavlink_prosight_osd_ignition_key_t packet;
	packet.key = key;
	packet.token = token;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PROSIGHT_OSD_IGNITION_KEY_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_PROSIGHT_OSD_IGNITION_KEY;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_PROSIGHT_OSD_IGNITION_KEY_MIN_LEN, MAVLINK_MSG_ID_PROSIGHT_OSD_IGNITION_KEY_LEN, MAVLINK_MSG_ID_PROSIGHT_OSD_IGNITION_KEY_CRC);
}

/**
 * @brief Pack a prosight_osd_ignition_key message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param key Key challenge/answer
 * @param token Key token
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_prosight_osd_ignition_key_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint32_t key,uint8_t token)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_PROSIGHT_OSD_IGNITION_KEY_LEN];
	_mav_put_uint32_t(buf, 0, key);
	_mav_put_uint8_t(buf, 4, token);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PROSIGHT_OSD_IGNITION_KEY_LEN);
#else
	mavlink_prosight_osd_ignition_key_t packet;
	packet.key = key;
	packet.token = token;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PROSIGHT_OSD_IGNITION_KEY_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_PROSIGHT_OSD_IGNITION_KEY;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_PROSIGHT_OSD_IGNITION_KEY_MIN_LEN, MAVLINK_MSG_ID_PROSIGHT_OSD_IGNITION_KEY_LEN, MAVLINK_MSG_ID_PROSIGHT_OSD_IGNITION_KEY_CRC);
}

/**
 * @brief Encode a prosight_osd_ignition_key struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param prosight_osd_ignition_key C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_prosight_osd_ignition_key_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_prosight_osd_ignition_key_t* prosight_osd_ignition_key)
{
	return mavlink_msg_prosight_osd_ignition_key_pack(system_id, component_id, msg, prosight_osd_ignition_key->key, prosight_osd_ignition_key->token);
}

/**
 * @brief Encode a prosight_osd_ignition_key struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param prosight_osd_ignition_key C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_prosight_osd_ignition_key_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_prosight_osd_ignition_key_t* prosight_osd_ignition_key)
{
	return mavlink_msg_prosight_osd_ignition_key_pack_chan(system_id, component_id, chan, msg, prosight_osd_ignition_key->key, prosight_osd_ignition_key->token);
}

/**
 * @brief Send a prosight_osd_ignition_key message
 * @param chan MAVLink channel to send the message
 *
 * @param key Key challenge/answer
 * @param token Key token
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_prosight_osd_ignition_key_send(mavlink_channel_t chan, uint32_t key, uint8_t token)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_PROSIGHT_OSD_IGNITION_KEY_LEN];
	_mav_put_uint32_t(buf, 0, key);
	_mav_put_uint8_t(buf, 4, token);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PROSIGHT_OSD_IGNITION_KEY, buf, MAVLINK_MSG_ID_PROSIGHT_OSD_IGNITION_KEY_MIN_LEN, MAVLINK_MSG_ID_PROSIGHT_OSD_IGNITION_KEY_LEN, MAVLINK_MSG_ID_PROSIGHT_OSD_IGNITION_KEY_CRC);
#else
	mavlink_prosight_osd_ignition_key_t packet;
	packet.key = key;
	packet.token = token;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PROSIGHT_OSD_IGNITION_KEY, (const char *)&packet, MAVLINK_MSG_ID_PROSIGHT_OSD_IGNITION_KEY_MIN_LEN, MAVLINK_MSG_ID_PROSIGHT_OSD_IGNITION_KEY_LEN, MAVLINK_MSG_ID_PROSIGHT_OSD_IGNITION_KEY_CRC);
#endif
}

/**
 * @brief Send a prosight_osd_ignition_key message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_prosight_osd_ignition_key_send_struct(mavlink_channel_t chan, const mavlink_prosight_osd_ignition_key_t* prosight_osd_ignition_key)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_prosight_osd_ignition_key_send(chan, prosight_osd_ignition_key->key, prosight_osd_ignition_key->token);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PROSIGHT_OSD_IGNITION_KEY, (const char *)prosight_osd_ignition_key, MAVLINK_MSG_ID_PROSIGHT_OSD_IGNITION_KEY_MIN_LEN, MAVLINK_MSG_ID_PROSIGHT_OSD_IGNITION_KEY_LEN, MAVLINK_MSG_ID_PROSIGHT_OSD_IGNITION_KEY_CRC);
#endif
}

#if MAVLINK_MSG_ID_PROSIGHT_OSD_IGNITION_KEY_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_prosight_osd_ignition_key_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t key, uint8_t token)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint32_t(buf, 0, key);
	_mav_put_uint8_t(buf, 4, token);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PROSIGHT_OSD_IGNITION_KEY, buf, MAVLINK_MSG_ID_PROSIGHT_OSD_IGNITION_KEY_MIN_LEN, MAVLINK_MSG_ID_PROSIGHT_OSD_IGNITION_KEY_LEN, MAVLINK_MSG_ID_PROSIGHT_OSD_IGNITION_KEY_CRC);
#else
	mavlink_prosight_osd_ignition_key_t *packet = (mavlink_prosight_osd_ignition_key_t *)msgbuf;
	packet->key = key;
	packet->token = token;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PROSIGHT_OSD_IGNITION_KEY, (const char *)packet, MAVLINK_MSG_ID_PROSIGHT_OSD_IGNITION_KEY_MIN_LEN, MAVLINK_MSG_ID_PROSIGHT_OSD_IGNITION_KEY_LEN, MAVLINK_MSG_ID_PROSIGHT_OSD_IGNITION_KEY_CRC);
#endif
}
#endif

#endif

// MESSAGE PROSIGHT_OSD_IGNITION_KEY UNPACKING


/**
 * @brief Get field key from prosight_osd_ignition_key message
 *
 * @return Key challenge/answer
 */
static inline uint32_t mavlink_msg_prosight_osd_ignition_key_get_key(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field token from prosight_osd_ignition_key message
 *
 * @return Key token
 */
static inline uint8_t mavlink_msg_prosight_osd_ignition_key_get_token(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Decode a prosight_osd_ignition_key message into a struct
 *
 * @param msg The message to decode
 * @param prosight_osd_ignition_key C-struct to decode the message contents into
 */
static inline void mavlink_msg_prosight_osd_ignition_key_decode(const mavlink_message_t* msg, mavlink_prosight_osd_ignition_key_t* prosight_osd_ignition_key)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	prosight_osd_ignition_key->key = mavlink_msg_prosight_osd_ignition_key_get_key(msg);
	prosight_osd_ignition_key->token = mavlink_msg_prosight_osd_ignition_key_get_token(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_PROSIGHT_OSD_IGNITION_KEY_LEN? msg->len : MAVLINK_MSG_ID_PROSIGHT_OSD_IGNITION_KEY_LEN;
        memset(prosight_osd_ignition_key, 0, MAVLINK_MSG_ID_PROSIGHT_OSD_IGNITION_KEY_LEN);
	memcpy(prosight_osd_ignition_key, _MAV_PAYLOAD(msg), len);
#endif
}
