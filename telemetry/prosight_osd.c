/*
 * prosight_osd.c
 *
 *  Created on: 19/09/2016
 *      Author: Shmulik Fridman
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "build_config.h"

#include "version.h"

#include "debug.h"

#ifdef TELEMETRY

#include "common/maths.h"
#include "common/axis.h"
#include "common/color.h"

#include "drivers/system.h"
#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/gpio.h"
#include "drivers/timer.h"
#include "drivers/serial.h"
#include "drivers/pwm_rx.h"


#include "sensors/sensors.h"
#include "sensors/acceleration.h"
#include "sensors/gyro.h"
#include "sensors/barometer.h"
#include "sensors/boardalignment.h"
#include "sensors/battery.h"
#include "sensors/sonar.h"


#include "io/serial.h"
#include "io/rc_controls.h"
#include "io/gimbal.h"
#include "io/gps.h"
#include "io/ledstrip.h"

#include "rx/rx.h"

#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/failsafe.h"
#include "flight/altitudehold.h"
#include "flight/navigation.h"
#include "flight/direction_mode.h"
#include "flight/crash_detection.h"
#include "flight/pos_estimation.h"
#include "flight/onground_detection.h"

#include "telemetry/prosight_osd.h"
#include "telemetry/telemetry.h"
#include "telemetry/mavlink.h"
#include "mavlink/common/mavlink.h"

#include "config/config.h"
#include "config/runtime_config.h"
#include "config/config_profile.h"
#include "config/config_master.h"


#define PROSIGHT_DONT_SHOW_VAL_INT8	    	0x7F
#define PROSIGHT_DONT_SHOW_VAL_INT16		0x7FFF
#define PROSIGHT_DONT_SHOW_VAL_INT32		0x7FFFFFFF

#define MAX_FAIL_RETRIES                    4 // 4 retries before we decide that the error is real and not a glitch

// general logic definitions
#define ALTITUDE_MAX_DISPLAY_10CM_UNITS     12 // max altitude to display in 10cm steps

#define THROTTLE_OFF						0
#define THROTTLE_MAX						100
#define THROTTLE_AAH_POS_QUANT				(THROTTLE_MAX / FALCORE_AAH_THR_POS_FULL)

extern estAltitude_s  g_estAltitude;

static uint8_t sonar_fail_ctr   = 0;


#ifdef USE_PROSIGHT_IGNITION

#define SECRET_AUTHENTICATION_KEY           0x23F42B1A      // A random number, for salting

// secret key is 8 bytes: CPU_ID_0 << 32 | SECRET_AUTHENTICATION_KEY
// Sending to ProSight CPU_ID_0, ProSight response is running JOAAT on hashData
static uint32_t hashData[2]                 = { 0, SECRET_AUTHENTICATION_KEY };
static bool     ignition_key_valid          = false;
static uint8_t  ignition_token              = 0;
#endif

static bool prosight_osd__is_shield_mode(void)
{
    return FLIGHT_MODE(ALTHOLD_DIRECT) &&  FLIGHT_MODE(DIRECTION_MODE) && FLIGHT_MODE (ANGLE_MODE);
}

int16_t	prosight_osd__get_altitude(void)
{
    int alt_data = 0;

    // shield mode
    if ( prosight_osd__is_shield_mode() )
    {
        alt_data = (int) constrainf( (g_estAltitude.est.aglAltitude+5.0) / 10.0 , 0, ALTITUDE_MAX_DISPLAY_10CM_UNITS+1) ;  // in 10cm steps, +5 is for rounding to nearest integer
        if ( alt_data > ALTITUDE_MAX_DISPLAY_10CM_UNITS
                || g_estAltitude.est.quality == EST_ALT_QUAL_DR)
        {
            return PROSIGHT_DONT_SHOW_VAL_INT16;
        }
        return (int16_t)alt_data;
    }

    // Acro/Horizon/SNL
    // TODO: rethink if this is the behavior we want in SNL
    else
    {
        // get raw sonar readings into 10cm steps
        alt_data = (int32_t) constrainf( (g_estAltitude.sonar.sonarRaw + 5.0) / 10.0, 0, ALTITUDE_MAX_DISPLAY_10CM_UNITS+1) ;
        // don't show if Sonar Invalid read OR sonar HW failure OR on ground OR above ALTITUDE_MAX_DISPLAY_10CM_UNITS
        if ( (g_estAltitude.sonar.sonarRaw == SONAR_OUT_OF_RANGE)
                ||  (g_estAltitude.sonar.sonarRaw == SONAR_HW_FAILURE)
                ||  (g_onGroundState == ONGROUND)
                ||  (alt_data > ALTITUDE_MAX_DISPLAY_10CM_UNITS) )
        {
            return PROSIGHT_DONT_SHOW_VAL_INT16;
        }
        return (int16_t)alt_data;
    }
    return PROSIGHT_DONT_SHOW_VAL_INT16;
}


falcore_msg_t prosight_osd__get_msg_data(void)
{
	falcore_msg_t msg_id;
	batteryState_e bat_state;

	bat_state = getBatteryState();

	msg_id.fields.priority 	= FALCORE_MSG_PRIORITY_NONE;
	msg_id.fields.marker 	= FALCORE_MSG_MARKER_NONE;

	// following messages order is the Falcore priority
	//	FALCORE_MSG_ID_ARMED
	if ( STATE(MOTOR_STOP) && ARMING_FLAG(ARMED) )
	{
		msg_id.fields.msg_id 	= FALCORE_MSG_ID_ARMED;
	}

	//	FALCORE_MSG_ID_PRODUCT_UNSIGNED
//	else if (0) //TBD
//	{
//	    msg_id.fields.msg_id 	= FALCORE_MSG_ID_PRODUCT_UNSIGNED;
//	}

#ifdef USE_PROSIGHT_IGNITION
	else if ( prosight_osd__is_ignition_key_valid() )
	{
	    msg_id.fields.msg_id    = FALCORE_MSG_ID_IGNITION_FAILED;
	}
#endif

	//	FALCORE_MSG_ID_SONAR_FAIL - only in shield mode
	else if ( (!sonarIsAvailable()) && prosight_osd__is_shield_mode() )
	{
	    // we make sure the error is active for MAX_FAIL_RETRIES times before we set it
	    sonar_fail_ctr++;
	    if (sonar_fail_ctr > MAX_FAIL_RETRIES)
	    {
	        msg_id.fields.msg_id 	= FALCORE_MSG_ID_SONAR_FAIL;
	    }
	}

	//	FALCORE_MSG_ID_BARO_FAIL
	else if (!sensors(SENSOR_BARO))
	{
	        msg_id.fields.msg_id 	= FALCORE_MSG_ID_BARO_FAIL;
	}

	//	FALCORE_MSG_ID_NEED_TO_CALIB_ACC
	else if( !isAccelerometerCalibrationValid() )
	{
		msg_id.fields.msg_id 	= FALCORE_MSG_ID_NEED_TO_CALIB_ACC;
	}

	//	FALCORE_MSG_ID_CALIBRATING
	else if ( !isBaroCalibrationComplete() )
	{
		msg_id.fields.msg_id 	= FALCORE_MSG_ID_CALIBRATING;
	}

	//	FALCORE_MSG_ID_CRITICAL_BAT
	else if (bat_state == BATTERY_CRITICAL && prosight_osd__is_shield_mode())
	{
		msg_id.fields.msg_id 	= FALCORE_MSG_ID_CRITICAL_BAT;
	}

	//	FALCORE_MSG_ID_LOW_BAT
	else if (bat_state == BATTERY_WARNING && prosight_osd__is_shield_mode())
	{
	    msg_id.fields.msg_id 	= FALCORE_MSG_ID_LOW_BAT;
	}

	// Failsafe
	else if ( ((feature(FEATURE_FAILSAFE) && failsafeIsActive()) || (!ARMING_FLAG(ARMED) & !failsafeIsReceivingRxData())) )
	{
	    msg_id.fields.msg_id = FALCORE_MSG_ID_NO_REMOTE_CONTROL_SIGNAL;
	}

	//	FALCORE_MSG_ID_CANNOT_ARM
	else if (!STATE(SMALL_ANGLE) && !ARMING_FLAG(ARMED) && !ARMING_FLAG(OK_TO_ARM))
	{
	    msg_id.fields.msg_id 	= FALCORE_MSG_ID_CANNOT_ARM;
	}

	// reserved for future use
	//	FALCORE_MSG_ID_E1
	//	FALCORE_MSG_ID_E2
	//	FALCORE_MSG_ID_E3
	//	FALCORE_MSG_ID_E4
	//	FALCORE_MSG_ID_E5
	//	FALCORE_MSG_ID_E6
	//	FALCORE_MSG_ID_E7
	//	FALCORE_MSG_ID_E8
	//	FALCORE_MSG_ID_E9

	else
	{
		msg_id.fields.msg_id 	= FALCORE_MSG_ID_NO_ERR;
		// if all OK, then reset consecutive error counters
		sonar_fail_ctr = 0;
	}

	return msg_id;
}


uint8_t prosight_osd__get_flight_mode(void)
{
    // in Amimon black remote, ARM and FLM switches are on the same channel
    // So, if not armed than FLM is STBY
    if ( ARMING_FLAG(ARMED) )
    {
        if (FLIGHT_MODE(SNL_MODE))
        {
            return FALCORE_FLIGHT_MODE_SNL;
        }
        else if ( prosight_osd__is_shield_mode() )
        {
            return FALCORE_FLIGHT_MODE_SHIELD1;
        }
        else if (FLIGHT_MODE(ALTHOLD_ASSIST) )
        {
            return FALCORE_FLIGHT_MODE_AAH;
        }
        else if (FLIGHT_MODE(HORIZON_MODE) )
        {
            return FALCORE_FLIGHT_MODE_HORIZON;
        }
        else if (FLIGHT_MODE(ANGLE_MODE)) {
            return 255;   //FIXME: relpace with FALCORE_FLIGHT_MODE_ANGLE
        }
        // if no other FLM, then ACRO
        return FALCORE_FLIGHT_MODE_ACRO;
    }

    return FALCORE_FLIGHT_MODE_STBY;
}

uint8_t prosight_osd__get_aah_throttle_position(void)
{
    uint8_t curr_thr    = 0;
    uint8_t ret_val     = FALCORE_AAH_THR_POS_OFF;

    // display only on shield2 (AAH)
    if ( FLIGHT_MODE(ALTHOLD_ASSIST) )
    {
        curr_thr = scaleRange(constrain(rcData[THROTTLE], PWM_RANGE_MIN, PWM_RANGE_MAX), PWM_RANGE_MIN, PWM_RANGE_MAX, 0, 100);

        switch (curr_thr)
        {
        case THROTTLE_OFF:
            ret_val = FALCORE_AAH_THR_POS_OFF;
            break;

        case THROTTLE_MAX:
            ret_val = FALCORE_AAH_THR_POS_FULL;
            break;

        default:
            // dynamic range of max values +1. e.g. if thr is on 35, then (int)(35/20) + 1 = 2 (low throttle pos).
            // debug[1] = (curr_thr / THROTTLE_AAH_POS_QUANT) + 1;
            ret_val = ((curr_thr / THROTTLE_AAH_POS_QUANT) + 1);
            break;
        }
    }

	return ret_val;
}


falcore_osd_flags_t prosight_osd__get_flags(void)
{
	falcore_osd_flags_t flags;

	// init
	flags.fields.hover 		= 0;
	flags.fields.reserved 	= 0;

	// hover
	if ( prosight_osd__is_shield_mode()
	        && (altitudeHoldGetEstimatedAltitude() > 0)
			&& (isDroneHovering() ) )
	{
		flags.fields.hover = 1;
	}

	return flags;
}


#ifdef USE_PROSIGHT_IGNITION
// Ignition

/**
 * @brief key generation function
 * @param ignition struct pointer with key values and message token number
 * @return none
 */
void prosight_osd__ignition_key_generate_challenge(falcore_ignition_t *response)
{
    // TODO replace with CoCPU challenge generation
    response->challenge = U_ID_0;
    response->token = ignition_token;
    ignition_token++;
    // endof TODO

    return;
}

/**
 * @brief key resolve function – checks if received answer to challenge is correct
 * and set ignition valid accordingly.
 * @param ignition struct pointer with key values and message token number
 * @return true if challenge answered correctly, otherwise returns false
 */
uint8_t prosight_osd__ignition_key_check_challenge_response(falcore_ignition_t *response)
{
    // TODO replace with CoCPU challenge response check
    // calculate expected response
    hashData[0] = U_ID_0;
    uint32_t authResponse = hashJOAAT((const uint8_t *)hashData, sizeof(hashData));

    // check response
    if ( response->challenge == authResponse)
    {
        return true;
    }
    // endof TODO

    return false;
}

/**
 * @brief returns if ignition key validation completed
 *
 * @return true if ignition key have been unlocked, return false if lock is active.
 */
bool prosight_osd__is_ignition_key_valid(void)
{
    return ignition_key_valid;
}

/**
 * @brief set of ignition key validation status according to challenge result
 * @param ignition key validation status (0 - invalid, 1-valid)
 */
void prosight_osd__set_ignition_key_status(uint8_t status)
{
    ignition_key_valid = status;
    return;
}
#endif // PROSIGHT_IGNITION

#endif
