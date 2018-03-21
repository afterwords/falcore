// MESSAGE PROSIGHT_OSD PACKING

#define MAVLINK_MSG_ID_PROSIGHT_OSD 150

MAVPACKED(
typedef struct __mavlink_prosight_osd_t {
 int16_t altitude; /*< Altitude in 10cm steps. value 0x7FFF->don't display, 0x7FFE->display "Max" msg*/
 uint16_t msg_id; /*< Active message ID. value 0x0->no active errors. see falcore_msg_struct_t*/
 uint8_t flags; /*< 8bit general flags. see 'falcore_flags_s' struct*/
 uint8_t flight_mode; /*< Falcore current flight mode. see FALCORE_FLIGHT_MODE enum*/
 uint8_t aah_thr_pos; /*< AAH flight mode throttle position. 0xFF->don't display*/
}) mavlink_prosight_osd_t;

#define MAVLINK_MSG_ID_PROSIGHT_OSD_LEN 7
#define MAVLINK_MSG_ID_PROSIGHT_OSD_MIN_LEN 7
#define MAVLINK_MSG_ID_150_LEN 7
#define MAVLINK_MSG_ID_150_MIN_LEN 7

#define MAVLINK_MSG_ID_PROSIGHT_OSD_CRC 156
#define MAVLINK_MSG_ID_150_CRC 156



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_PROSIGHT_OSD { \
	150, \
	"PROSIGHT_OSD", \
	5, \
	{  { "altitude", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_prosight_osd_t, altitude) }, \
         { "msg_id", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_prosight_osd_t, msg_id) }, \
         { "flags", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_prosight_osd_t, flags) }, \
         { "flight_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_prosight_osd_t, flight_mode) }, \
         { "aah_thr_pos", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_prosight_osd_t, aah_thr_pos) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_PROSIGHT_OSD { \
	"PROSIGHT_OSD", \
	5, \
	{  { "altitude", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_prosight_osd_t, altitude) }, \
         { "msg_id", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_prosight_osd_t, msg_id) }, \
         { "flags", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_prosight_osd_t, flags) }, \
         { "flight_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_prosight_osd_t, flight_mode) }, \
         { "aah_thr_pos", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_prosight_osd_t, aah_thr_pos) }, \
         } \
}
#endif

/**
 * @brief Pack a prosight_osd message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param altitude Altitude in 10cm steps. value 0x7FFF->don't display, 0x7FFE->display "Max" msg
 * @param msg_id Active message ID. value 0x0->no active errors. see falcore_msg_struct_t
 * @param flags 8bit general flags. see 'falcore_flags_s' struct
 * @param flight_mode Falcore current flight mode. see FALCORE_FLIGHT_MODE enum
 * @param aah_thr_pos AAH flight mode throttle position. 0xFF->don't display
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_prosight_osd_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       int16_t altitude, uint16_t msg_id, uint8_t flags, uint8_t flight_mode, uint8_t aah_thr_pos)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_PROSIGHT_OSD_LEN];
	_mav_put_int16_t(buf, 0, altitude);
	_mav_put_uint16_t(buf, 2, msg_id);
	_mav_put_uint8_t(buf, 4, flags);
	_mav_put_uint8_t(buf, 5, flight_mode);
	_mav_put_uint8_t(buf, 6, aah_thr_pos);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PROSIGHT_OSD_LEN);
#else
	mavlink_prosight_osd_t packet;
	packet.altitude = altitude;
	packet.msg_id = msg_id;
	packet.flags = flags;
	packet.flight_mode = flight_mode;
	packet.aah_thr_pos = aah_thr_pos;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PROSIGHT_OSD_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_PROSIGHT_OSD;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_PROSIGHT_OSD_MIN_LEN, MAVLINK_MSG_ID_PROSIGHT_OSD_LEN, MAVLINK_MSG_ID_PROSIGHT_OSD_CRC);
}

/**
 * @brief Pack a prosight_osd message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param altitude Altitude in 10cm steps. value 0x7FFF->don't display, 0x7FFE->display "Max" msg
 * @param msg_id Active message ID. value 0x0->no active errors. see falcore_msg_struct_t
 * @param flags 8bit general flags. see 'falcore_flags_s' struct
 * @param flight_mode Falcore current flight mode. see FALCORE_FLIGHT_MODE enum
 * @param aah_thr_pos AAH flight mode throttle position. 0xFF->don't display
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_prosight_osd_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           int16_t altitude,uint16_t msg_id,uint8_t flags,uint8_t flight_mode,uint8_t aah_thr_pos)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_PROSIGHT_OSD_LEN];
	_mav_put_int16_t(buf, 0, altitude);
	_mav_put_uint16_t(buf, 2, msg_id);
	_mav_put_uint8_t(buf, 4, flags);
	_mav_put_uint8_t(buf, 5, flight_mode);
	_mav_put_uint8_t(buf, 6, aah_thr_pos);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PROSIGHT_OSD_LEN);
#else
	mavlink_prosight_osd_t packet;
	packet.altitude = altitude;
	packet.msg_id = msg_id;
	packet.flags = flags;
	packet.flight_mode = flight_mode;
	packet.aah_thr_pos = aah_thr_pos;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PROSIGHT_OSD_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_PROSIGHT_OSD;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_PROSIGHT_OSD_MIN_LEN, MAVLINK_MSG_ID_PROSIGHT_OSD_LEN, MAVLINK_MSG_ID_PROSIGHT_OSD_CRC);
}

/**
 * @brief Encode a prosight_osd struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param prosight_osd C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_prosight_osd_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_prosight_osd_t* prosight_osd)
{
	return mavlink_msg_prosight_osd_pack(system_id, component_id, msg, prosight_osd->altitude, prosight_osd->msg_id, prosight_osd->flags, prosight_osd->flight_mode, prosight_osd->aah_thr_pos);
}

/**
 * @brief Encode a prosight_osd struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param prosight_osd C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_prosight_osd_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_prosight_osd_t* prosight_osd)
{
	return mavlink_msg_prosight_osd_pack_chan(system_id, component_id, chan, msg, prosight_osd->altitude, prosight_osd->msg_id, prosight_osd->flags, prosight_osd->flight_mode, prosight_osd->aah_thr_pos);
}

/**
 * @brief Send a prosight_osd message
 * @param chan MAVLink channel to send the message
 *
 * @param altitude Altitude in 10cm steps. value 0x7FFF->don't display, 0x7FFE->display "Max" msg
 * @param msg_id Active message ID. value 0x0->no active errors. see falcore_msg_struct_t
 * @param flags 8bit general flags. see 'falcore_flags_s' struct
 * @param flight_mode Falcore current flight mode. see FALCORE_FLIGHT_MODE enum
 * @param aah_thr_pos AAH flight mode throttle position. 0xFF->don't display
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_prosight_osd_send(mavlink_channel_t chan, int16_t altitude, uint16_t msg_id, uint8_t flags, uint8_t flight_mode, uint8_t aah_thr_pos)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_PROSIGHT_OSD_LEN];
	_mav_put_int16_t(buf, 0, altitude);
	_mav_put_uint16_t(buf, 2, msg_id);
	_mav_put_uint8_t(buf, 4, flags);
	_mav_put_uint8_t(buf, 5, flight_mode);
	_mav_put_uint8_t(buf, 6, aah_thr_pos);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PROSIGHT_OSD, buf, MAVLINK_MSG_ID_PROSIGHT_OSD_MIN_LEN, MAVLINK_MSG_ID_PROSIGHT_OSD_LEN, MAVLINK_MSG_ID_PROSIGHT_OSD_CRC);
#else
	mavlink_prosight_osd_t packet;
	packet.altitude = altitude;
	packet.msg_id = msg_id;
	packet.flags = flags;
	packet.flight_mode = flight_mode;
	packet.aah_thr_pos = aah_thr_pos;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PROSIGHT_OSD, (const char *)&packet, MAVLINK_MSG_ID_PROSIGHT_OSD_MIN_LEN, MAVLINK_MSG_ID_PROSIGHT_OSD_LEN, MAVLINK_MSG_ID_PROSIGHT_OSD_CRC);
#endif
}

/**
 * @brief Send a prosight_osd message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_prosight_osd_send_struct(mavlink_channel_t chan, const mavlink_prosight_osd_t* prosight_osd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_prosight_osd_send(chan, prosight_osd->altitude, prosight_osd->msg_id, prosight_osd->flags, prosight_osd->flight_mode, prosight_osd->aah_thr_pos);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PROSIGHT_OSD, (const char *)prosight_osd, MAVLINK_MSG_ID_PROSIGHT_OSD_MIN_LEN, MAVLINK_MSG_ID_PROSIGHT_OSD_LEN, MAVLINK_MSG_ID_PROSIGHT_OSD_CRC);
#endif
}

#if MAVLINK_MSG_ID_PROSIGHT_OSD_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_prosight_osd_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  int16_t altitude, uint16_t msg_id, uint8_t flags, uint8_t flight_mode, uint8_t aah_thr_pos)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_int16_t(buf, 0, altitude);
	_mav_put_uint16_t(buf, 2, msg_id);
	_mav_put_uint8_t(buf, 4, flags);
	_mav_put_uint8_t(buf, 5, flight_mode);
	_mav_put_uint8_t(buf, 6, aah_thr_pos);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PROSIGHT_OSD, buf, MAVLINK_MSG_ID_PROSIGHT_OSD_MIN_LEN, MAVLINK_MSG_ID_PROSIGHT_OSD_LEN, MAVLINK_MSG_ID_PROSIGHT_OSD_CRC);
#else
	mavlink_prosight_osd_t *packet = (mavlink_prosight_osd_t *)msgbuf;
	packet->altitude = altitude;
	packet->msg_id = msg_id;
	packet->flags = flags;
	packet->flight_mode = flight_mode;
	packet->aah_thr_pos = aah_thr_pos;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PROSIGHT_OSD, (const char *)packet, MAVLINK_MSG_ID_PROSIGHT_OSD_MIN_LEN, MAVLINK_MSG_ID_PROSIGHT_OSD_LEN, MAVLINK_MSG_ID_PROSIGHT_OSD_CRC);
#endif
}
#endif

#endif

// MESSAGE PROSIGHT_OSD UNPACKING


/**
 * @brief Get field altitude from prosight_osd message
 *
 * @return Altitude in 10cm steps. value 0x7FFF->don't display, 0x7FFE->display "Max" msg
 */
static inline int16_t mavlink_msg_prosight_osd_get_altitude(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  0);
}

/**
 * @brief Get field msg_id from prosight_osd message
 *
 * @return Active message ID. value 0x0->no active errors. see falcore_msg_struct_t
 */
static inline uint16_t mavlink_msg_prosight_osd_get_msg_id(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Get field flags from prosight_osd message
 *
 * @return 8bit general flags. see 'falcore_flags_s' struct
 */
static inline uint8_t mavlink_msg_prosight_osd_get_flags(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field flight_mode from prosight_osd message
 *
 * @return Falcore current flight mode. see FALCORE_FLIGHT_MODE enum
 */
static inline uint8_t mavlink_msg_prosight_osd_get_flight_mode(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field aah_thr_pos from prosight_osd message
 *
 * @return AAH flight mode throttle position. 0xFF->don't display
 */
static inline uint8_t mavlink_msg_prosight_osd_get_aah_thr_pos(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Decode a prosight_osd message into a struct
 *
 * @param msg The message to decode
 * @param prosight_osd C-struct to decode the message contents into
 */
static inline void mavlink_msg_prosight_osd_decode(const mavlink_message_t* msg, mavlink_prosight_osd_t* prosight_osd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	prosight_osd->altitude = mavlink_msg_prosight_osd_get_altitude(msg);
	prosight_osd->msg_id = mavlink_msg_prosight_osd_get_msg_id(msg);
	prosight_osd->flags = mavlink_msg_prosight_osd_get_flags(msg);
	prosight_osd->flight_mode = mavlink_msg_prosight_osd_get_flight_mode(msg);
	prosight_osd->aah_thr_pos = mavlink_msg_prosight_osd_get_aah_thr_pos(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_PROSIGHT_OSD_LEN? msg->len : MAVLINK_MSG_ID_PROSIGHT_OSD_LEN;
        memset(prosight_osd, 0, MAVLINK_MSG_ID_PROSIGHT_OSD_LEN);
	memcpy(prosight_osd, _MAV_PAYLOAD(msg), len);
#endif
}
