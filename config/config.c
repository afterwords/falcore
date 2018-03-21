                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  /*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "build_config.h"

#include "common/color.h"
#include "common/axis.h"
#include "common/maths.h"

#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/compass.h"
#include "drivers/system.h"
#include "drivers/gpio.h"
#include "drivers/timer.h"
#include "drivers/pwm_rx.h"
#include "drivers/serial.h"

#include "sensors/sensors.h"
#include "sensors/gyro.h"
#include "sensors/compass.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/boardalignment.h"
#include "sensors/battery.h"

#include "io/beeper.h"
#include "io/serial.h"
#include "io/gimbal.h"
#include "io/escservo.h"
#include "io/rc_controls.h"
#include "io/rc_curves.h"
#include "io/ledstrip.h"
#include "io/gps.h"

#include "rx/rx.h"

#include "blackbox/blackbox_io.h"

#include "telemetry/telemetry.h"

#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/failsafe.h"
#include "flight/altitudehold.h"
#include "flight/direction_mode.h"
#include "flight/navigation.h"
#include "flight/crash_detection.h"

#include "config/runtime_config.h"
#include "config/config.h"

#include "config/config_profile.h"
#include "config/config_master.h"

#define BRUSHED_MOTORS_PWM_RATE 16000
#define BRUSHLESS_MOTORS_PWM_RATE 400

void useRcControlsConfig(modeActivationCondition_t *modeActivationConditions, escAndServoConfig_t *escAndServoConfigToUse, pidProfile_t *pidProfileToUse);

#if !defined(FLASH_SIZE)
#error "Flash size not defined for target. (specify in KB)"
#endif


#ifndef FLASH_PAGE_SIZE
    #ifdef STM32F303xC
        #define FLASH_PAGE_SIZE                 ((uint16_t)0x800)
    #endif

    #ifdef STM32F10X_MD
        #define FLASH_PAGE_SIZE                 ((uint16_t)0x400)
    #endif

    #ifdef STM32F10X_HD
        #define FLASH_PAGE_SIZE                 ((uint16_t)0x800)
    #endif
#endif

#if !defined(FLASH_SIZE) && !defined(FLASH_PAGE_COUNT)
    #ifdef STM32F10X_MD
        #define FLASH_PAGE_COUNT 128
    #endif

    #ifdef STM32F10X_HD
        #define FLASH_PAGE_COUNT 128
    #endif
#endif

#if defined(FLASH_SIZE)
#define FLASH_PAGE_COUNT ((FLASH_SIZE * 0x400) / FLASH_PAGE_SIZE)
#endif

#if !defined(FLASH_PAGE_SIZE)
#error "Flash page size not defined for target."
#endif

#if !defined(FLASH_PAGE_COUNT)
#error "Flash page count not defined for target."
#endif

#if FLASH_SIZE <= 64
#define FLASH_TO_RESERVE_FOR_CONFIG 0x0800
#else
#define FLASH_TO_RESERVE_FOR_CONFIG 0x1000
#endif

// use the last flash pages for storage
#define CONFIG_START_FLASH_ADDRESS (0x08000000 + (uint32_t)((FLASH_PAGE_SIZE * FLASH_PAGE_COUNT) - FLASH_TO_RESERVE_FOR_CONFIG))

master_t masterConfig;                 // master config struct with data independent from profiles
profile_t *currentProfile;
static uint32_t activeFeaturesLatch = 0;

static uint8_t currentControlRateProfileIndex = 0;
controlRateConfig_t *currentControlRateProfile;

static const uint8_t EEPROM_CONF_VERSION = 130;

/** PROFILE 0 template *******************/
static const profile_t profile_Template = {
    .pidProfile = { 
        .P8[ROLL] = 62,
        .I8[ROLL] = 40,
        .D8[ROLL] = 60,
        .P8[PITCH] = 80,
        .I8[PITCH] = 50,
        .D8[PITCH] = 90,
        .P8[YAW] = 124,
        .I8[YAW] = 20,
        .D8[YAW] = 0,
        .P8[PIDLEVEL] = 72,
        .I8[PIDLEVEL] = 72,
        .D8[PIDLEVEL] = 75,
        .P8[PIDALT] = 15,
        .I8[PIDALT] = 0,
        .D8[PIDALT] = 0,
        .P8[PIDVEL] = 24,
        .I8[PIDVEL] = 100,
        .D8[PIDVEL] = 13,
        .P8[PIDPOS] = 15,
        .I8[PIDPOS] = 0,
        .D8[PIDPOS] = 0,
        .P8[PIDPOSR] = 34,
        .I8[PIDPOSR] = 14,
        .D8[PIDPOSR] = 53,
        .P8[PIDNAVR] = 25,
        .I8[PIDNAVR] = 33,
        .D8[PIDNAVR] = 83,
        .P8[PIDMAG] = 40,

        .acc_soft_lpf_hz = 10,
        .gyro_soft_lpf_hz = 70,
        .dterm_lpf_hz = 55,

        .rollPitchItermIgnoreRate = 200,
        .yawItermIgnoreRate = 50,

        .max_angle_inclination[ROLL][0] = 300,
        .max_angle_inclination[ROLL][1] = 300,
        .max_angle_inclination[PITCH][0] = 220,
        .max_angle_inclination[PITCH][1] = 220,
    },
    .defaultRateProfileIndex = 0,
    .mag_declination = 0,

#if BUILDTYPE == ARF
    .modeActivationConditions = {
        { BOXARM,           1,  { PWM_TO_RANGE(1500),   PWM_TO_RANGE(2100) } },
        { BOXSNL,         2,  { PWM_TO_RANGE(1500),   PWM_TO_RANGE(2100) } },
        { BOXMODELOCK,      0,  { PWM_TO_RANGE(900),    PWM_TO_RANGE(1300) } },
        { BOXANGLE,         0,  { PWM_TO_RANGE(900),    PWM_TO_RANGE(1300) } },
        { BOXHORIZON,       0,  { PWM_TO_RANGE(1300),   PWM_TO_RANGE(1700) } },
        { BOXAHDIRECT,      0,  { PWM_TO_RANGE(900),    PWM_TO_RANGE(1300) } },
        { BOXRATE1,         0,  { PWM_TO_RANGE(900),    PWM_TO_RANGE(1300) } },
        { BOXRATE2,         0,  { PWM_TO_RANGE(1300),   PWM_TO_RANGE(1700) } },
        { BOXRATE3,         0,  { PWM_TO_RANGE(1700),   PWM_TO_RANGE(2100) } },
        { BOXDIRECTION_YAW, 0,  { PWM_TO_RANGE(900),    PWM_TO_RANGE(1300) } },
    },
#else   // R&D and RTF
    .modeActivationConditions = {
        { BOXARM,           0,  { PWM_TO_RANGE(1125),   PWM_TO_RANGE(2100) } },
        //{ BOXAIRMODE,       0,  { PWM_TO_RANGE(1125),   PWM_TO_RANGE(2100) } },
        { BOXSNL,         0,  { PWM_TO_RANGE(1900),   PWM_TO_RANGE(2100) } },
        { BOXMODELOCK,      0,  { PWM_TO_RANGE(1125),   PWM_TO_RANGE(1300) } },
        { BOXANGLE,         0,  { PWM_TO_RANGE(900),    PWM_TO_RANGE(1300) } },
        { BOXHORIZON,       0,  { PWM_TO_RANGE(1300),   PWM_TO_RANGE(1700) } },
        { BOXAHDIRECT,      0,  { PWM_TO_RANGE(1125),   PWM_TO_RANGE(1300) } },
        { BOXRATE1,         0,  { PWM_TO_RANGE(1125),    PWM_TO_RANGE(1300) } },
        { BOXRATE2,         0,  { PWM_TO_RANGE(1300),   PWM_TO_RANGE(1700) } },
        { BOXRATE3,         0,  { PWM_TO_RANGE(1700),   PWM_TO_RANGE(2100) } },
        { BOXDIRECTION_YAW, 0,  { PWM_TO_RANGE(1125),   PWM_TO_RANGE(1300) } },
        //{ BOXBEEPERON,      2,  { PWM_TO_RANGE(1250),   PWM_TO_RANGE(1750) } },
    },
#endif
    .adjustmentRanges = {
        { 0 },
    },
    .rcControlsConfig = { 
        .deadband = 1,
        .yaw_deadband = 2,
    },
    .throttle_tilt_compensation_strength = 75,
    .dirModeSettings = {
        // AMIMON: Direction mode parameters
        .droneMass = 740,
        .extraRoll = 100,
        .stickExtraRoll = 33,
        .dirModeValidVel = 100,
        .yawToRollRatio = 0.7f,
        .yawStickFactor = 1.0f
    },
    .altHoldSettings = {
        .sonar_pitch_angle = 18,
        .hover_throttle = 1345,
        .sonar_max_alt = 100,
        .sonar_range = 250,
        .emergency_descent_timeout = 2.5f,
        .baro_max_alt = 2000,
        .alt_baro_z_w = 0.750f,
        .alt_baro_v_w = 0.280f,
        .alt_sonar_z_w = 3.000f,
        .alt_sonar_v_w = 4.700f,
        .alt_baro_accz_bias_w = 0.002f,
        .alt_hold_deadband = 100,
        .max_roc_rod = 100,
        .max_snl_rod = 250,
    },
/*     .crashDetectionSettings = {
        .mode = 0,
    }, */
#ifdef GPS
    .gpsProfile = {
        .gps_wp_radius = 200,
        .gps_lpf = 20,
        .nav_slew_rate = 30,
        .nav_controls_heading = 1,
        .nav_speed_min = 100,
        .nav_speed_max = 300,
        .ap_mode = 40
    },
#endif
};

static const controlRateConfig_t controlRateConfig_Template[3] = {
    {
        /* Control rate profile 0 SH1 */
        .rcRate8 = 100,
        .rcExpo8 = 65,
        .thrMid8 = 50,
        .thrExpo8 = 0,
        .dynThrPID = 20,
        .rcYawExpo8 = 10,
        .tpa_breakpoint = 1500,
        .rates[FD_ROLL] = 60,
        .rates[FD_PITCH] = 60,
        .rates[FD_YAW] = 15,
        .yaw_p_limit = 200,
    },
    {
        /* Control rate profile 1 HORIZON */
        .rcRate8 = 100,
        .rcExpo8 = 85,
        .thrMid8 = 50,
        .thrExpo8 = 0,
        .dynThrPID = 20,
        .rcYawExpo8 = 10,
        .tpa_breakpoint = 1500,
        .rates[FD_ROLL] = 40,
        .rates[FD_PITCH] = 40,
        .rates[FD_YAW] = 20,
        .yaw_p_limit = 200,
    },
    {
        /* Control rate profile 2 ACRO */
        .rcRate8 = 100,
        .rcExpo8 = 85,
        .thrMid8 = 50,
        .thrExpo8 = 0,
        .dynThrPID = 20,
        .rcYawExpo8 = 10,
        .tpa_breakpoint = 1500,
        .rates[FD_ROLL] = 40,
        .rates[FD_PITCH] = 40,
        .rates[FD_YAW] = 20,
        .yaw_p_limit = 350,
    },
};



static void resetAccelerometerCalibrationData(flightDynamicsTrims_t * accZero, flightDynamicsTrims_t * accGain)
{
    accZero->values.pitch = 0;
    accZero->values.roll = 0;
    accZero->values.yaw = 0;

    accGain->values.pitch = 4096;
    accGain->values.roll = 4096;
    accGain->values.yaw = 4096;
}

static void resetProfileFromTemplate(profile_t * profile, const profile_t * template)
{
    // Copy template
    memcpy(profile, template, sizeof(profile_t));

#ifdef USE_SERVOS
    // servos
    for (int i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        profile->servoConf[i].min = DEFAULT_SERVO_MIN;
        profile->servoConf[i].max = DEFAULT_SERVO_MAX;
        profile->servoConf[i].middle = DEFAULT_SERVO_MIDDLE;
        profile->servoConf[i].rate = 100;
        profile->servoConf[i].angleAtMin = DEFAULT_SERVO_MIN_ANGLE;
        profile->servoConf[i].angleAtMax = DEFAULT_SERVO_MAX_ANGLE;
        profile->servoConf[i].forwardFromChannel = CHANNEL_FORWARDING_DISABLED;
    }

    // gimbal
    profile->gimbalConfig.mode = GIMBAL_MODE_NORMAL;
#endif
}

static void resetControlRateConfigFromTemplate(controlRateConfig_t * controlRateConfig, const controlRateConfig_t * template)
{
    memcpy(controlRateConfig, template, sizeof(controlRateConfig_t));
}

void resetBarometerConfig(barometerConfig_t *barometerConfig)
{
    barometerConfig->apply_median_filter = 1;
}

void resetSensorAlignment(sensorAlignmentConfig_t *sensorAlignmentConfig)
{
    sensorAlignmentConfig->gyro_align = ALIGN_DEFAULT;
    sensorAlignmentConfig->acc_align = ALIGN_DEFAULT;
    sensorAlignmentConfig->mag_align = ALIGN_DEFAULT;
}

void resetEscAndServoConfig(escAndServoConfig_t *escAndServoConfig)
{
    escAndServoConfig->minthrottle = 1050;
    escAndServoConfig->maxthrottle = 2000;
    escAndServoConfig->mincommand = 1000;
    escAndServoConfig->servoCenterPulse = 1500;
}

void resetFlight3DConfig(flight3DConfig_t *flight3DConfig)
{
    flight3DConfig->deadband3d_low = 1406;
    flight3DConfig->deadband3d_high = 1514;
    flight3DConfig->neutral3d = 1460;
    flight3DConfig->deadband3d_throttle = 50;
}

void resetTelemetryConfig(telemetryConfig_t *telemetryConfig)
{
    telemetryConfig->telemetry_inversion = 0;
    telemetryConfig->telemetry_switch = 0;
    telemetryConfig->gpsNoFixLatitude = 0;
    telemetryConfig->gpsNoFixLongitude = 0;
    telemetryConfig->frsky_coordinate_format = FRSKY_FORMAT_DMS;
    telemetryConfig->frsky_unit = FRSKY_UNIT_IMPERIALS;
    telemetryConfig->frsky_vfas_precision = 0;
    telemetryConfig->hottAlarmSoundInterval = 5;
}

void resetBatteryConfig(batteryConfig_t *batteryConfig)
{
    batteryConfig->vbatscale = VBAT_SCALE_DEFAULT;
    batteryConfig->vbatresdivval = VBAT_RESDIVVAL_DEFAULT;
    batteryConfig->vbatresdivmultiplier = VBAT_RESDIVMULTIPLIER_DEFAULT;
    batteryConfig->vbatmaxcellvoltage = 43;
    batteryConfig->vbatmincellvoltage = 33;
    batteryConfig->vbatwarningcellvoltage = 35;
    batteryConfig->currentMeterOffset = 0;
    batteryConfig->currentMeterScale = 400; // for Allegro ACS758LCB-100U (40mV/A)
    batteryConfig->batteryCapacity = 0;
    batteryConfig->currentMeterType = CURRENT_SENSOR_ADC;
}

#ifdef SWAP_SERIAL_PORT_0_AND_1_DEFAULTS
#define FIRST_PORT_INDEX 1
#define SECOND_PORT_INDEX 0
#else
#define FIRST_PORT_INDEX 0
#define SECOND_PORT_INDEX 1
#endif

void resetSerialConfig(serialConfig_t *serialConfig)
{
    uint8_t index;
    memset(serialConfig, 0, sizeof(serialConfig_t));

    for (index = 0; index < SERIAL_PORT_COUNT; index++) {
        serialConfig->portConfigs[index].identifier = serialPortIdentifiers[index];
        serialConfig->portConfigs[index].msp_baudrateIndex = BAUD_115200;
        serialConfig->portConfigs[index].gps_baudrateIndex = BAUD_57600;
        serialConfig->portConfigs[index].telemetry_baudrateIndex = BAUD_AUTO;
        serialConfig->portConfigs[index].blackbox_baudrateIndex = BAUD_250000;
    }

    serialConfig->portConfigs[0].functionMask = FUNCTION_MSP;

#ifdef CC3D
    // This allows MSP connection via USART & VCP so the board can be reconfigured.
    serialConfig->portConfigs[1].functionMask = FUNCTION_MSP;
#endif

    serialConfig->reboot_character = 'R';
}

void resetMixerConfig(mixerConfig_t *mixerConfig) {
    mixerConfig->yaw_motor_direction = 1;
    mixerConfig->yaw_jump_prevention_limit = 200;
#ifdef USE_SERVOS
    mixerConfig->tri_unarmed_servo = 1;
    mixerConfig->servo_lowpass_freq = 400;
    mixerConfig->servo_lowpass_enable = 0;
#endif
}

uint8_t getCurrentProfile(void)
{
    return masterConfig.current_profile_index;
}

static void setProfile(uint8_t profileIndex)
{
    currentProfile = &masterConfig.profile[profileIndex];
}

uint8_t getCurrentControlRateProfile(void)
{
    return currentControlRateProfileIndex;
}

controlRateConfig_t *getControlRateConfig(uint8_t profileIndex) {
    return &masterConfig.controlRateProfiles[profileIndex];
}

static void setControlRateProfile(uint8_t profileIndex)
{
    currentControlRateProfileIndex = profileIndex;
    currentControlRateProfile = &masterConfig.controlRateProfiles[profileIndex];
}

uint16_t getCurrentMinthrottle(void)
{
    return masterConfig.escAndServoConfig.minthrottle;
}

static bool testAndBackupCalibrationData(masterCalibration_t * backup, const master_t * masterOrigin)
{
    bool isCalibrationDataValid = (masterOrigin->calibration.calibrationMagic == 0x464C4352) && 
                                  (masterOrigin->calibrationCRC == hashJOAAT((uint8_t *)&masterOrigin->calibration, sizeof(masterCalibration_t)));

    if (isCalibrationDataValid) {
        memcpy(backup, &masterOrigin->calibration, sizeof(masterCalibration_t));
    }

    return isCalibrationDataValid;
}

// Default settings
static void resetConf(void)
{
    int i;
    masterCalibration_t backupCalibration;
    bool isCalibrationDataValid = false;

    // 1-st attempts, validate data from run-time config
    // This will backup from run-time values and will succeed when there is a MSP/CLI request to reset calibration
    isCalibrationDataValid = testAndBackupCalibrationData(&backupCalibration, &masterConfig);

    // 2-nd attempt, try to get calibration data straight from EEPROM
    // This will succeed if configuration structure changes, but EEPROM calibration data might still be valid
    if (!isCalibrationDataValid) {
        isCalibrationDataValid = testAndBackupCalibrationData(&backupCalibration, (const master_t *)CONFIG_START_FLASH_ADDRESS);
    }

    // Clear all configuration
    memset(&masterConfig, 0, sizeof(master_t));

    setProfile(0);
    setControlRateProfile(0);

    masterConfig.version = EEPROM_CONF_VERSION;
    masterConfig.mixerMode = MIXER_QUADX;
    featureClearAll();
#if defined(CJMCU) || defined(SPARKY) || defined(COLIBRI_RACE) || defined(MOTOLAB)
    featureSet(FEATURE_RX_PPM);
#endif

#ifdef BOARD_HAS_VOLTAGE_DIVIDER
    // only enable the VBAT feature by default if the board has a voltage divider otherwise
    // the user may see incorrect readings and unexpected issues with pin mappings may occur.
    featureSet(FEATURE_VBAT);
#endif

    featureSet(FEATURE_FAILSAFE);

    // global settings
    masterConfig.current_profile_index = 0;     // default profile

    // AMIMON: New IMU gains
    masterConfig.dcm_kp_acc = 2500;             // 0.25 * 10000
    masterConfig.dcm_ki_acc = 30;               // 0.00 * 10000
    masterConfig.dcm_kp_mag = 10000;            // 1.00 * 10000
    masterConfig.dcm_ki_mag = 30;               // 0.00 * 10000
    masterConfig.gyro_lpf = 1;                  // 188HZ (MPU6500)

    resetAccelerometerCalibrationData(&masterConfig.calibration.accZero, &masterConfig.calibration.accGain);

    resetSensorAlignment(&masterConfig.sensorAlignmentConfig);

    masterConfig.calibration.boardAlignment.rollDeciDegrees = 0;
    masterConfig.calibration.boardAlignment.pitchDeciDegrees = 170;
    masterConfig.calibration.boardAlignment.yawDeciDegrees = 0;
    masterConfig.acc_hardware = ACC_DEFAULT;     // default/autodetect
    masterConfig.yaw_control_direction = 1;
    masterConfig.gyroConfig.gyroMovementCalibrationThreshold = 32;

    masterConfig.mag_hardware = MAG_DEFAULT;     // default/autodetect
    masterConfig.baro_hardware = BARO_DEFAULT;   // default/autodetect

    resetBatteryConfig(&masterConfig.batteryConfig);

    resetTelemetryConfig(&masterConfig.telemetryConfig);

    masterConfig.rxConfig.serialrx_provider = 0;
    masterConfig.rxConfig.spektrum_sat_bind = 0;
    masterConfig.rxConfig.midrc = 1500;
    masterConfig.rxConfig.mincheck = 1020;
    masterConfig.rxConfig.maxcheck = 1900;
    masterConfig.rxConfig.rx_min_usec = 885;          // any of first 4 channels below this value will trigger rx loss detection
    masterConfig.rxConfig.rx_max_usec = 2115;         // any of first 4 channels above this value will trigger rx loss detection

    for (i = 0; i < MAX_SUPPORTED_RC_CHANNEL_COUNT; i++) {
        rxFailsafeChannelConfiguration_t *channelFailsafeConfiguration = &masterConfig.rxConfig.failsafe_channel_configurations[i];
        // Set yaw/pitch/roll to center stick position (AUTO). Throttle stick is not set - held at last known good value (HOLD)
        channelFailsafeConfiguration->mode = (i < NON_AUX_CHANNEL_COUNT && i != THROTTLE) ? RX_FAILSAFE_MODE_AUTO : RX_FAILSAFE_MODE_HOLD;
        channelFailsafeConfiguration->step = (i == THROTTLE) ? CHANNEL_VALUE_TO_RXFAIL_STEP(masterConfig.rxConfig.rx_min_usec) : CHANNEL_VALUE_TO_RXFAIL_STEP(masterConfig.rxConfig.midrc);
    }

    masterConfig.rxConfig.rssi_channel = 0;
    masterConfig.rxConfig.rssi_scale = RSSI_SCALE_DEFAULT;
    masterConfig.rxConfig.rssi_ppm_invert = 0;
    masterConfig.rxConfig.rcSmoothing = 1;

    resetAllRxChannelRangeConfigurations(masterConfig.rxConfig.channelRanges);

    masterConfig.inputFilteringMode = INPUT_FILTERING_DISABLED;

    masterConfig.disarm_kill_switch = 1;
    masterConfig.auto_disarm_delay = 30;
    masterConfig.small_angle = 60;

    resetMixerConfig(&masterConfig.mixerConfig);

    masterConfig.airplaneConfig.fixedwing_althold_dir = 1;

    // Motor/ESC/Servo
    resetEscAndServoConfig(&masterConfig.escAndServoConfig);
    resetFlight3DConfig(&masterConfig.flight3DConfig);

#ifdef BRUSHED_MOTORS
    masterConfig.motor_pwm_rate = BRUSHED_MOTORS_PWM_RATE;
#else
    masterConfig.motor_pwm_rate = BRUSHLESS_MOTORS_PWM_RATE;
#endif
    masterConfig.servo_pwm_rate = 50;

#ifdef GPS
    // gps/nav stuff
    masterConfig.gpsConfig.provider = GPS_UBLOX;
    masterConfig.gpsConfig.sbasMode = SBAS_AUTO;
    masterConfig.gpsConfig.autoConfig = GPS_AUTOCONFIG_ON;
    masterConfig.gpsConfig.autoBaud = GPS_AUTOBAUD_OFF;
#endif

    resetSerialConfig(&masterConfig.serialConfig);

    masterConfig.looptime = 1000;
    masterConfig.emf_avoidance = 0;
    masterConfig.i2c_overclock = 0;
    masterConfig.gyroSync = 1;
    masterConfig.gyroSyncDenominator = 1;

    // Reset profiles
    resetProfileFromTemplate(&masterConfig.profile[0], &profile_Template);
    resetControlRateConfigFromTemplate(&masterConfig.controlRateProfiles[0], &controlRateConfig_Template[0]);

    resetProfileFromTemplate(&masterConfig.profile[1], &profile_Template);
    resetControlRateConfigFromTemplate(&masterConfig.controlRateProfiles[1], &controlRateConfig_Template[1]);

    resetProfileFromTemplate(&masterConfig.profile[2], &profile_Template);
    resetControlRateConfigFromTemplate(&masterConfig.controlRateProfiles[2], &controlRateConfig_Template[2]);

    // Make sure Rate profiles are linked to Profiles
    for (i = 1; i < MAX_PROFILE_COUNT; i++) {
        masterConfig.profile[i].defaultRateProfileIndex = i % MAX_CONTROL_RATE_PROFILE_COUNT;
    }

    resetBarometerConfig(&masterConfig.barometerConfig);

    // Radio
    parseRcChannels("AETR1234", &masterConfig.rxConfig);

    // Failsafe Variables
    masterConfig.failsafeConfig.failsafe_procedure = FAILSAFE_PROCEDURE_SNL;
    masterConfig.failsafeConfig.failsafe_delay = 2;               // 200ms
    masterConfig.failsafeConfig.failsafe_off_delay = 200;         // 20sec
    masterConfig.failsafeConfig.failsafe_throttle = 1200;         // default throttle to 20% (below hover in any case but above MOTOR_STOP).
    masterConfig.failsafeConfig.failsafe_kill_switch = 0;         // default failsafe switch action is identical to rc link loss
    masterConfig.failsafeConfig.failsafe_throttle_low_delay = 100; // default throttle low delay for "just disarm" on failsafe condition

    // custom mixer. clear by defaults.
    for (i = 0; i < MAX_SUPPORTED_MOTORS; i++)
        masterConfig.customMotorMixer[i].throttle = 0.0f;

#ifdef LED_STRIP
    applyDefaultColors(masterConfig.colors, CONFIGURABLE_COLOR_COUNT);
    applyDefaultLedStripConfig(masterConfig.ledConfigs);
#endif

#ifdef BLACKBOX
#ifdef ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT
    masterConfig.blackbox_device = BLACKBOX_DEVICE_FLASH;
#else
    masterConfig.blackbox_device = BLACKBOX_DEVICE_SERIAL;
#endif

#if BUILDTYPE == RND
    featureSet(FEATURE_BLACKBOX);
#else
    featureClear(FEATURE_BLACKBOX);
#endif

    masterConfig.blackbox_rate_num = 1;
    masterConfig.blackbox_rate_denom = 4;
#endif

    //AMIMON: Specific configuration
#if defined(CONNEXFC)
    masterConfig.looptime = 1000;
    masterConfig.gyroSync = 1;
    masterConfig.gyroSyncDenominator = 1;

    masterConfig.escAndServoConfig.minthrottle = 1050;
    masterConfig.escAndServoConfig.maxthrottle = 2000;

    masterConfig.failsafeConfig.failsafe_delay = 2;
    masterConfig.failsafeConfig.failsafe_off_delay = 0;

    featureClear(FEATURE_RX_PPM);
    featureClear(FEATURE_RX_PARALLEL_PWM);
    featureClear(FEATURE_RX_MSP);
    featureSet(FEATURE_RX_SERIAL);
    featureSet(FEATURE_VBAT);
    featureSet(FEATURE_RSSI_ADC);
    featureSet(FEATURE_ONESHOT125);
    featureSet(FEATURE_LED_STRIP);
    featureSet(FEATURE_MOTOR_STOP);
    featureSet(FEATURE_SONAR);
    featureSet(FEATURE_TELEMETRY);

    masterConfig.mag_hardware = 1;
    masterConfig.auto_disarm_delay = 30;
    
    masterConfig.batteryConfig.vbatscale = 78;
    masterConfig.rxConfig.rssi_scale = 20;

#if BUILDTYPE == ARF
    masterConfig.rxConfig.serialrx_provider = SERIALRX_SBUS;
#else
    masterConfig.rxConfig.serialrx_provider = SERIALRX_IBUS;
#endif

    masterConfig.enableBuzzer = 1;

    masterConfig.serialConfig.portConfigs[0].functionMask = FUNCTION_MSP;                               // VCP: MSP
#if BUILDTYPE == RND
    masterConfig.serialConfig.portConfigs[1].functionMask = FUNCTION_BLACKBOX;                          // UART1: OpenLog
#endif
    masterConfig.serialConfig.portConfigs[2].functionMask = FUNCTION_RX_SERIAL;                         // UART2: SerialRX
    masterConfig.serialConfig.portConfigs[3].functionMask = FUNCTION_TELEMETRY_SMARTPORT;               // UART4: Smartport telemetry
    masterConfig.serialConfig.portConfigs[4].functionMask = FUNCTION_TELEMETRY_MAVLINK;                 // UART5: MAVLink telemetry

#ifdef USE_PROSIGHT_IGNITION
    masterConfig.enableProsightIgnition = 1;
#endif

#endif

    // Restore calibration
    if (isCalibrationDataValid) {
        memcpy(&masterConfig.calibration, &backupCalibration, sizeof(masterCalibration_t));
    }
}

static uint8_t calculateChecksum(const uint8_t *data, uint32_t length)
{
    uint8_t checksum = 0;
    const uint8_t *byteOffset;

    for (byteOffset = data; byteOffset < (data + length); byteOffset++)
        checksum ^= *byteOffset;
    return checksum;
}

static bool isEEPROMContentValid(void)
{
    const master_t *temp = (const master_t *) CONFIG_START_FLASH_ADDRESS;
    uint8_t checksum = 0;

    // check version number
    if (EEPROM_CONF_VERSION != temp->version)
        return false;

    // check size and magic numbers
    if (temp->size != sizeof(master_t) || temp->magic_be != 0xBE || temp->magic_ef != 0xEF)
        return false;

    // verify integrity of temporary copy
    checksum = calculateChecksum((const uint8_t *) temp, sizeof(master_t));
    if (checksum != 0)
        return false;

    // looks good, let's roll!
    return true;
}

void activateControlRateConfig(void)
{
    generatePitchRollCurve(currentControlRateProfile);
    generateYawCurve(currentControlRateProfile);
    generateThrottleCurve(currentControlRateProfile, &masterConfig.escAndServoConfig);
}

void activateConfig(void)
{
    static imuRuntimeConfig_t imuRuntimeConfig;

    activateControlRateConfig();

    resetAdjustmentStates();

    pidResetCustomInclinationLimits();

    useRcControlsConfig(
        currentProfile->modeActivationConditions,
        &masterConfig.escAndServoConfig,
        &currentProfile->pidProfile
    );

    useGyroConfig(&masterConfig.gyroConfig, currentProfile->pidProfile.gyro_soft_lpf_hz);

#ifdef TELEMETRY
    telemetryUseConfig(&masterConfig.telemetryConfig);
#endif

#ifdef GPS
    gpsUseProfile(&currentProfile->gpsProfile);
    gpsUsePIDs(&currentProfile->pidProfile);
#endif

    useFailsafeConfig(&masterConfig.failsafeConfig);

    setAccelerationZero(&masterConfig.calibration.accZero);
    setAccelerationGain(&masterConfig.calibration.accGain);
    setAccelerationFilter(currentProfile->pidProfile.acc_soft_lpf_hz);

	g_rxConfig = &masterConfig.rxConfig;
    mixerUseConfigs(
#ifdef USE_SERVOS
        currentProfile->servoConf,
        &currentProfile->gimbalConfig,
#endif
        &masterConfig.flight3DConfig,
        &masterConfig.escAndServoConfig,
        &masterConfig.mixerConfig,
        &masterConfig.airplaneConfig,
        &masterConfig.rxConfig
    );

    imuRuntimeConfig.dcm_kp_acc = masterConfig.dcm_kp_acc / 10000.0f;
    imuRuntimeConfig.dcm_ki_acc = masterConfig.dcm_ki_acc / 10000.0f;
    imuRuntimeConfig.dcm_kp_mag = masterConfig.dcm_kp_mag / 10000.0f;   // FIXME: Use separate gains here
    imuRuntimeConfig.dcm_ki_mag = masterConfig.dcm_ki_mag / 10000.0f;   // FIXME: Use separate gains here
    imuRuntimeConfig.small_angle = masterConfig.small_angle;

    imuConfigure(
        &imuRuntimeConfig,
        &currentProfile->pidProfile
    );

#ifdef BARO
    configureAltitudeHold(
        &currentProfile->pidProfile,
        &currentProfile->altHoldSettings,
        &masterConfig.escAndServoConfig,
        &masterConfig.rxConfig);
#endif

    configureDirectionMode(&currentProfile->dirModeSettings, &masterConfig.rxConfig, &currentProfile->pidProfile);

    //configureCrashDetection(&currentProfile->crashDetectionSettings);

#ifdef BARO
    useBarometerConfig(&masterConfig.barometerConfig);
#endif
}

void validateAndFixConfig(void)
{
    if (!(featureConfigured(FEATURE_RX_PARALLEL_PWM) || featureConfigured(FEATURE_RX_PPM) || featureConfigured(FEATURE_RX_SERIAL) || featureConfigured(FEATURE_RX_MSP))) {
        featureSet(FEATURE_RX_PARALLEL_PWM); // Consider changing the default to PPM
    }

    if (featureConfigured(FEATURE_RX_PPM)) {
        featureClear(FEATURE_RX_PARALLEL_PWM);
    }

    if (featureConfigured(FEATURE_RX_MSP)) {
        featureClear(FEATURE_RX_SERIAL);
        featureClear(FEATURE_RX_PARALLEL_PWM);
        featureClear(FEATURE_RX_PPM);
    }

    if (featureConfigured(FEATURE_RX_SERIAL)) {
        featureClear(FEATURE_RX_PARALLEL_PWM);
        featureClear(FEATURE_RX_PPM);
    }

    if (featureConfigured(FEATURE_RX_PARALLEL_PWM)) {
#if defined(STM32F10X)
        // rssi adc needs the same ports
        featureClear(FEATURE_RSSI_ADC);
        // current meter needs the same ports
        if (masterConfig.batteryConfig.currentMeterType == CURRENT_SENSOR_ADC) {
            featureClear(FEATURE_CURRENT_METER);
        }
#endif

#if defined(STM32F10X) || defined(CHEBUZZ) || defined(STM32F3DISCOVERY)
        // led strip needs the same ports
        featureClear(FEATURE_LED_STRIP);
#endif

        // software serial needs free PWM ports
        featureClear(FEATURE_SOFTSERIAL);
    }


#if defined(LED_STRIP) && (defined(USE_SOFTSERIAL1) || defined(USE_SOFTSERIAL2))
    if (featureConfigured(FEATURE_SOFTSERIAL) && (
            0
#ifdef USE_SOFTSERIAL1
            || (LED_STRIP_TIMER == SOFTSERIAL_1_TIMER)
#endif
#ifdef USE_SOFTSERIAL2
            || (LED_STRIP_TIMER == SOFTSERIAL_2_TIMER)
#endif
    )) {
        // led strip needs the same timer as softserial
        featureClear(FEATURE_LED_STRIP);
    }
#endif

#if defined(CC3D) && defined(DISPLAY) && defined(USE_USART3)
    if (doesConfigurationUsePort(SERIAL_PORT_USART3) && feature(FEATURE_DISPLAY)) {
        featureClear(FEATURE_DISPLAY);
    }
#endif

#ifdef STM32F303xC
    // hardware supports serial port inversion, make users life easier for those that want to connect SBus RX's
    masterConfig.telemetryConfig.telemetry_inversion = 1;
#endif

#if defined(CC3D) && defined(SONAR) && defined(USE_SOFTSERIAL1)
    if (feature(FEATURE_SONAR) && feature(FEATURE_SOFTSERIAL)) {
        featureClear(FEATURE_SONAR);
    }
#endif

#if defined(COLIBRI_RACE)
    masterConfig.serialConfig.portConfigs[0].functionMask = FUNCTION_MSP;
    if(featureConfigured(FEATURE_RX_SERIAL)) {
        masterConfig.serialConfig.portConfigs[2].functionMask = FUNCTION_RX_SERIAL;
    }
#endif

    // AMIMON FC
#if defined(CONNEXFC)
    // That's USB - always MSP
    masterConfig.serialConfig.portConfigs[0].functionMask = FUNCTION_MSP;

    // UART2 - The only SerialRX capable port
    if (featureConfigured(FEATURE_RX_SERIAL)) {
	    masterConfig.serialConfig.portConfigs[2].functionMask = FUNCTION_RX_SERIAL;
    }
#endif

    useRxConfig(&masterConfig.rxConfig);

    serialConfig_t *serialConfig = &masterConfig.serialConfig;

    if (!isSerialConfigValid(serialConfig)) {
        resetSerialConfig(serialConfig);
    }
}

void applyAndSaveBoardAlignmentDelta(int16_t roll, int16_t pitch)
{
    updateBoardAlignment(&masterConfig.calibration.boardAlignment, roll, pitch);

    saveConfigAndNotify();
}

void initEEPROM(void)
{
}

void readEEPROM(void)
{
    // Sanity check
    if (!isEEPROMContentValid())
        failureMode(FAILURE_INVALID_EEPROM_CONTENTS);

    suspendRxSignal();

    // Read flash
    memcpy(&masterConfig, (char *) CONFIG_START_FLASH_ADDRESS, sizeof(master_t));

    if (masterConfig.current_profile_index > MAX_PROFILE_COUNT - 1) // sanity check
        masterConfig.current_profile_index = 0;

    setProfile(masterConfig.current_profile_index);

    if (currentProfile->defaultRateProfileIndex > MAX_CONTROL_RATE_PROFILE_COUNT - 1) // sanity check
        currentProfile->defaultRateProfileIndex = 0;

    setControlRateProfile(currentProfile->defaultRateProfileIndex);

    validateAndFixConfig();
    activateConfig();

    resumeRxSignal();
}

void readEEPROMAndNotify(void)
{
    // re-read written data
    readEEPROM();
    beeperConfirmationBeeps(1);
}

void writeEEPROM(void)
{
    // Generate compile time error if the config does not fit in the reserved area of flash.
    BUILD_BUG_ON(sizeof(master_t) > FLASH_TO_RESERVE_FOR_CONFIG);

    FLASH_Status status = 0;
    uint32_t wordOffset;
    int8_t attemptsRemaining = 3;

    suspendRxSignal();

    // prepare checksum/version constants
    masterConfig.version = EEPROM_CONF_VERSION;
    masterConfig.size = sizeof(master_t);
    masterConfig.magic_be = 0xBE;
    masterConfig.magic_ef = 0xEF;
    masterConfig.calibration.calibrationMagic = 0x464C4352; // FLCR
    masterConfig.calibrationCRC = hashJOAAT((uint8_t *)&masterConfig.calibration, sizeof(masterCalibration_t));
    masterConfig.chk = 0; // erase checksum before recalculating
    masterConfig.chk = calculateChecksum((const uint8_t *) &masterConfig, sizeof(master_t));

    // write it
    FLASH_Unlock();
    while (attemptsRemaining--) {
#ifdef STM32F303
        FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);
#endif
#ifdef STM32F10X
        FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
#endif
        for (wordOffset = 0; wordOffset < sizeof(master_t); wordOffset += 4) {
            if (wordOffset % FLASH_PAGE_SIZE == 0) {
                status = FLASH_ErasePage(CONFIG_START_FLASH_ADDRESS + wordOffset);
                if (status != FLASH_COMPLETE) {
                    break;
                }
            }

            status = FLASH_ProgramWord(CONFIG_START_FLASH_ADDRESS + wordOffset,
                    *(uint32_t *) ((char *) &masterConfig + wordOffset));
            if (status != FLASH_COMPLETE) {
                break;
            }
        }
        if (status == FLASH_COMPLETE) {
            break;
        }
    }
    FLASH_Lock();

    // Flash write failed - just die now
    if (status != FLASH_COMPLETE || !isEEPROMContentValid()) {
        failureMode(FAILURE_FLASH_WRITE_FAILED);
    }

    resumeRxSignal();
}

void ensureEEPROMContainsValidData(void)
{
    if (isEEPROMContentValid()) {
        return;
    }

    resetEEPROM();
}

void resetEEPROM(void)
{
    resetConf();
    writeEEPROM();
}

void saveConfigAndNotify(void)
{
    writeEEPROM();
    readEEPROMAndNotify();
}

void changeProfile(uint8_t profileIndex)
{
    if (profileIndex == masterConfig.current_profile_index)
        return;

    masterConfig.current_profile_index = profileIndex;
    writeEEPROM();
    readEEPROM();
    beeperConfirmationBeeps(profileIndex + 1);
}

void changeControlRateProfile(uint8_t profileIndex)
{
    if (profileIndex > MAX_CONTROL_RATE_PROFILE_COUNT) {
        profileIndex = MAX_CONTROL_RATE_PROFILE_COUNT - 1;
    }
    setControlRateProfile(profileIndex);
    activateControlRateConfig();
}

void handleOneshotFeatureChangeOnRestart(void)
{
    // Shutdown PWM on all motors prior to soft restart
    StopPwmAllMotors();
    delay(50);
    // Apply additional delay when OneShot125 feature changed from on to off state
    if (feature(FEATURE_ONESHOT125) && !featureConfigured(FEATURE_ONESHOT125)) {
        delay(ONESHOT_FEATURE_CHANGED_DELAY_ON_BOOT_MS);
    }
}

void latchActiveFeatures()
{
    activeFeaturesLatch = masterConfig.enabledFeatures;
}

bool featureConfigured(uint32_t mask)
{
    return masterConfig.enabledFeatures & mask;
}

bool feature(uint32_t mask)
{
    return activeFeaturesLatch & mask;
}

void featureSet(uint32_t mask)
{
    masterConfig.enabledFeatures |= mask;
}

void featureClear(uint32_t mask)
{
    masterConfig.enabledFeatures &= ~(mask);
}

void featureClearAll()
{
    masterConfig.enabledFeatures = 0;
}

uint32_t featureMask(void)
{
    return masterConfig.enabledFeatures;
}

uint8_t getBeeperEnabledStaus(void)
{
    return masterConfig.enableBuzzer;
}

#ifdef USE_PROSIGHT_IGNITION
uint8_t getProsightIgnitionStatus(void)
{
    return masterConfig.enableProsightIgnition;
}
#endif
