/*
 * prosight_osd.h
 *
 *  Created on: 19 בספט׳ 2016
 *      Author: shmulikf
 */

#ifndef SRC_MAIN_TELEMETRY_PROSIGHT_OSD_H_
#define SRC_MAIN_TELEMETRY_PROSIGHT_OSD_H_

#include "platform.h"

// version, IDs etc
#define PROSIGHT_OSD_PROTOCOL_VERSION_MAJOR	0
#define PROSIGHT_OSD_PROTOCOL_VERSION_MINOR	1
#define FALCORE_AMIMON_VENDOR_ID			0x22CA
#define FALCORE_AMIMON_PRODUCT_ID			0x613F

typedef union
{
	struct
	{
		uint8_t hover		    : 1;
		uint8_t reserved	    : 7;
	}fields ;
	uint8_t bits;
} __attribute__ ((__packed__)) falcore_flags_t ;

typedef union
{
	struct
	{
		uint8_t sw_ver_minor    : 8;
		uint8_t sw_ver_major    : 8;
		uint8_t sw_board        : 8; // RND/ARF/ATF
		uint8_t family          : 8;
	}fields ;
	uint32_t bits;
} __attribute__ ((__packed__)) falcore_ver_t ;

typedef union
{
	struct
	{
		uint8_t minor    : 8;
		uint8_t major    : 8;
	}fields ;
	uint16_t bits;
} __attribute__ ((__packed__)) prosight_protocol_ver_t ;

typedef union
{
	struct
	{
		uint8_t hover		: 1;
		uint8_t reserved	: 7 ; // future use
	}fields;
	uint8_t bits;
} __attribute__ ((__packed__)) falcore_osd_flags_t ;


typedef union
{
	struct
	{
		uint8_t msg_id 	 : 8; // FALCORE_MSG_ID
		uint8_t priority : 2; // FALCORE_MSG_PRIORITY enum
		uint8_t marker 	 : 2; // FALCORE_MSG_MARKER enum
		uint8_t reserved : 4;
	}fields;
	uint16_t bits;
}__attribute__ ((__packed__)) falcore_msg_t ;

int16_t				prosight_osd__get_altitude(void);
falcore_msg_t 		prosight_osd__get_msg_data(void);
uint8_t 			prosight_osd__get_flight_mode(void);
uint8_t 			prosight_osd__get_aah_throttle_position(void);
falcore_osd_flags_t prosight_osd__get_flags(void);


#ifdef USE_PROSIGHT_IGNITION
// Ignition

typedef struct
{
    uint32_t challenge;
    uint8_t token;
}__attribute__ ((__packed__)) falcore_ignition_t;


/**
 * @brief key generation function
 * @param ignition struct pointer with key values and message token number
 * @return none
 */
void prosight_osd__ignition_key_generate_challenge(falcore_ignition_t *response);
/**
 * @brief key resolve function – checks if received answer to challenge is correct
 * and set ignition valid accordingly.
 * @param ignition struct pointer with key values and message token number
 * @return true if challenge answered correctly, otherwise returns false
 */
uint8_t prosight_osd__ignition_key_check_challenge_response(falcore_ignition_t *response);

/**
 * @brief returns if ignition key validation completed
 *
 * @return true if ignition key lock have been released, return false if lock is active.
 */
bool prosight_osd__is_ignition_key_valid(void);

/**
 * @brief set of ignition key validation status according to challenge result
 * @param ignition key validation status (0 - invalid, 1-valid)
 */
void prosight_osd__set_ignition_key_status(uint8_t status);
#endif // PROSIGHT_IGNITION

#endif /* SRC_MAIN_TELEMETRY_PROSIGHT_OSD_H_ */
