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
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

#include "platform.h"

#include "common/maths.h"
#include "common/axis.h"
#include "common/color.h"
#include "common/utils.h"
#include "common/filter.h"

#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/compass.h"
#include "drivers/light_led.h"

#include "drivers/gpio.h"
#include "drivers/system.h"
#include "drivers/serial.h"
#include "drivers/timer.h"
#include "drivers/pwm_rx.h"
#include "drivers/gyro_sync.h"

#include "sensors/sensors.h"
#include "sensors/boardalignment.h"
#include "sensors/sonar.h"
#include "sensors/compass.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/gyro.h"
#include "sensors/battery.h"

#include "io/beeper.h"
#include "io/display.h"
#include "io/escservo.h"
#include "io/rc_controls.h"
#include "io/rc_curves.h"
#include "io/gimbal.h"
#include "io/gps.h"
#include "io/ledstrip.h"
#include "io/serial.h"
#include "io/serial_cli.h"
#include "io/serial_msp.h"
#include "io/statusindicator.h"
#include "io/flashfs.h"

#include "rx/rx.h"
#include "rx/msp.h"

#include "telemetry/telemetry.h"
#include "blackbox/blackbox.h"

#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/altitudehold.h"
#include "flight/direction_mode.h"
#include "flight/panic_mode.h"
#include "flight/failsafe.h"
#include "flight/gtune.h"
#include "flight/navigation.h"
#include "flight/crash_detection.h"
#include "flight/pos_estimation.h"
#include "flight/onground_detection.h"

#include "config/runtime_config.h"
#include "config/config.h"
#include "config/config_profile.h"
#include "config/config_master.h"

#include "debug.h"
//#define DEBUG_THROTTLE

// June 2013     V2.2-dev

enum {
    ALIGN_GYRO = 0,
    ALIGN_ACCEL = 1,
    ALIGN_MAG = 2
};

/* VBAT monitoring interval (in microseconds) - 1s*/
#define VBATINTERVAL (6 * 3500)
/* IBat monitoring interval (in microseconds) - 6 default looptimes */
#define IBATINTERVAL (6 * 3500)
#define GYRO_WATCHDOG_DELAY 100  // Watchdog for boards without interrupt for gyro

uint32_t currentTime = 0;
uint32_t previousTime = 0;
uint16_t cycleTime = 0;         // this is the number in micro second to achieve a full loop, it can differ a little and is taken into account in the PID loop
float dT;

int16_t magHold;
int16_t headFreeModeHold;

uint8_t motorControlEnable = false;

int16_t telemTemperature1;      // gyro sensor temperature
static uint32_t disarmAt;     // Time of automatic disarm when "Don't spin the motors when armed" is enabled and auto_disarm_delay is nonzero

extern uint8_t dynP8[3], dynI8[3], dynD8[3], PIDweight[3];

static bool isRXDataNew;

void pidController(pidProfile_t *pidProfile, controlRateConfig_t *controlRateConfig, rxConfig_t *rxConfig);



#ifdef GTUNE

void updateGtuneState(void)
{
    static bool GTuneWasUsed = false;

    if (IS_RC_MODE_ACTIVE(BOXGTUNE)) {
        if (!FLIGHT_MODE(GTUNE_MODE) && ARMING_FLAG(ARMED)) {
            ENABLE_FLIGHT_MODE(GTUNE_MODE);
            init_Gtune(&currentProfile->pidProfile);
            GTuneWasUsed = true;
        }
        if (!FLIGHT_MODE(GTUNE_MODE) && !ARMING_FLAG(ARMED) && GTuneWasUsed) {
            saveConfigAndNotify();
            GTuneWasUsed = false;
        }
    } else {
        if (FLIGHT_MODE(GTUNE_MODE) && ARMING_FLAG(ARMED)) {
            DISABLE_FLIGHT_MODE(GTUNE_MODE);
        }
    }
}
#endif

#define HARDWARE_FAILURE_DETECTION_TIMEOUT_MS   200
bool isHardwareFailing(void)
{
    static uint32_t hardwareFailureTimer = 0;
    bool hardwareIsFailing = false;

    // CHECK 1 : Check for failing sonar
    if (!hardwareIsFailing) {
        hardwareIsFailing = sensors(SENSOR_SONAR) && !sonarIsAvailable() && (IS_RC_MODE_ACTIVE(BOXAHDIRECT) || IS_RC_MODE_ACTIVE(BOXAHASSIST));
    }

    // CHECK 2 : Check for unavailable ACC/BARO
    if (!hardwareIsFailing) {
        hardwareIsFailing = !sensors(SENSOR_ACC) || !sensors(SENSOR_BARO);
    }

    // CHECK 3: Check for uncalibrated accel
    if (!hardwareIsFailing) {
        hardwareIsFailing = !isAccelerometerCalibrationValid();
    }

		 // CHECK 4 : Check for failing sonar
    if (!hardwareIsFailing) {
        hardwareIsFailing = (g_crashState > NOT_CRASHED);
    }
	
    // No hardware falure at the moment - reset the timer
    if (!hardwareIsFailing) {
        hardwareFailureTimer = millis();
    }


	
    return ((millis() - hardwareFailureTimer) >= HARDWARE_FAILURE_DETECTION_TIMEOUT_MS);
}

bool isCalibrating(void)
{
    if (sensors(SENSOR_BARO) && !isBaroCalibrationComplete()) {
        return true;
    }

    if (!isImuReady()) {
        return true;
    }

    return false;
}

typedef enum {
    FLASHFS_WAITING,
    FLASHFS_CHECKING,
    FLASHFS_WAIT_FOR_READY,
    FLASHFS_READY
} dataflashPreparingState_e;

#define FLASHFS_MAX_FILL_PERCENT    70

bool isPreparingDataflash(void)
{
#ifdef USE_FLASHFS
    static dataflashPreparingState_e state = FLASHFS_WAITING;
    const flashGeometry_t * geometry;

    switch (state) {
    case FLASHFS_WAITING:
        // Wait for 5 seconds - safety measure to avoid accidental erasure 
        if (currentTime > 5 * 1000 * 1000) {
            state = FLASHFS_CHECKING;
        }
        return true;

    case FLASHFS_CHECKING:
        geometry = flashfsGetGeometry();
        if (geometry->totalSize > 0) {
            if (flashfsGetOffset() > geometry->totalSize * FLASHFS_MAX_FILL_PERCENT / 100) {
                // Flash is filled at 70% or more - check is it is safe to erase it
                if (feature(FEATURE_VBAT) && (batteryCellCount >= 3)) {
                    // Erase only if we are powered by LiPo battery of 3+ cells - this means that we won't erase the flash if powered by USB
                    flashfsEraseCompletely();
                    state = FLASHFS_WAIT_FOR_READY;
                }
                else {
                    state = FLASHFS_READY;
                }
            }
            else {
                state = FLASHFS_READY;
            }
        }
        else {
            // No flash chip found
            state = FLASHFS_READY;
        }
        return true;

    case FLASHFS_READY:
        // Preparation done
        return false;

    case FLASHFS_WAIT_FOR_READY:
    default:
        if (flashfsIsReady()) {
            state = FLASHFS_READY;
        }
        return true;
    }
#else
    return false;
#endif
}

void annexCode(void)
{
    static bool shouldAnnounceArmingReady = true;

    int32_t tmp, tmp2;
    int32_t axis, prop1 = 0, prop2;

    static uint32_t vbatLastServiced = 0;
    static uint32_t ibatLastServiced = 0;
    // PITCH & ROLL only dynamic PID adjustment,  depending on throttle value
    if (rcData[THROTTLE] < currentControlRateProfile->tpa_breakpoint) {
        prop2 = 100;
    } else {
        if (rcData[THROTTLE] < 2000) {
            prop2 = 100 - (uint16_t)currentControlRateProfile->dynThrPID * (rcData[THROTTLE] - currentControlRateProfile->tpa_breakpoint) / (2000 - currentControlRateProfile->tpa_breakpoint);
        } else {
            prop2 = 100 - currentControlRateProfile->dynThrPID;
        }
    }

    for (axis = 0; axis < 3; axis++) {
        tmp = MIN(ABS(rcData[axis] - masterConfig.rxConfig.midrc), 500);
        if (axis == ROLL || axis == PITCH) {
            if (currentProfile->rcControlsConfig.deadband) {
                if (tmp > currentProfile->rcControlsConfig.deadband) {
                    tmp -= currentProfile->rcControlsConfig.deadband;
                } else {
                    tmp = 0;
                }
            }

            tmp2 = tmp / 100;
            rcCommand[axis] = lookupPitchRollRC[tmp2] + (tmp - tmp2 * 100) * (lookupPitchRollRC[tmp2 + 1] - lookupPitchRollRC[tmp2]) / 100;
            prop1 = 100 - (uint16_t)currentControlRateProfile->rates[axis] * tmp / 500;
            prop1 = (uint16_t)prop1 * prop2 / 100;
        } else if (axis == YAW) {
            if (currentProfile->rcControlsConfig.yaw_deadband) {
                if (tmp > currentProfile->rcControlsConfig.yaw_deadband) {
                    tmp -= currentProfile->rcControlsConfig.yaw_deadband;
                } else {
                    tmp = 0;
                }
            }
            tmp2 = tmp / 100;
            rcCommand[axis] = (lookupYawRC[tmp2] + (tmp - tmp2 * 100) * (lookupYawRC[tmp2 + 1] - lookupYawRC[tmp2]) / 100) * -masterConfig.yaw_control_direction;
            prop1 = 100 - (uint16_t)currentControlRateProfile->rates[axis] * ABS(tmp) / 500;
        }
        // FIXME axis indexes into pids.  use something like lookupPidIndex(rc_alias_e alias) to reduce coupling.
        dynP8[axis] = (uint16_t)currentProfile->pidProfile.P8[axis] * prop1 / 100;
        dynI8[axis] = (uint16_t)currentProfile->pidProfile.I8[axis] * prop1 / 100;
        dynD8[axis] = (uint16_t)currentProfile->pidProfile.D8[axis] * prop1 / 100;

        // non coupled PID reduction scaler used in PID controller 1 and PID controller 2. YAW TPA disabled. 100 means 100% of the pids
        if (axis == YAW) {
            PIDweight[axis] = 100;
        }
        else {
            PIDweight[axis] = prop2;
        }

        if (rcData[axis] < masterConfig.rxConfig.midrc)
            rcCommand[axis] = -rcCommand[axis];
    }

    tmp = constrain(rcData[THROTTLE], masterConfig.rxConfig.mincheck, PWM_RANGE_MAX);
    tmp = (uint32_t)(tmp - masterConfig.rxConfig.mincheck) * PWM_RANGE_MIN / (PWM_RANGE_MAX - masterConfig.rxConfig.mincheck);       // [MINCHECK;2000] -> [0;1000]
    tmp2 = tmp / 100;
    rcCommand[THROTTLE] = lookupThrottleRC[tmp2] + (tmp - tmp2 * 100) * (lookupThrottleRC[tmp2 + 1] - lookupThrottleRC[tmp2]) / 100;    // [0;1000] -> expo -> [MINTHROTTLE;MAXTHROTTLE]

    /* At this point we applied all expo curves to rcCommand, but they still resemble actual pilot input without any corrections */
    for (axis = 0; axis < 4; axis++)
        rcStickCommand[axis] = rcCommand[axis];

    if (FLIGHT_MODE(HEADFREE_MODE)) {
        float radDiff = degreesToRadians(DECIDEGREES_TO_DEGREES(attitude.values.yaw) - headFreeModeHold);
        float cosDiff = cos_approx(radDiff);
        float sinDiff = sin_approx(radDiff);
        int16_t rcCommand_PITCH = rcCommand[PITCH] * cosDiff + rcCommand[ROLL] * sinDiff;
        rcCommand[ROLL] = rcCommand[ROLL] * cosDiff - rcCommand[PITCH] * sinDiff;
        rcCommand[PITCH] = rcCommand_PITCH;
    }

    if (feature(FEATURE_VBAT)) {
        if (cmp32(currentTime, vbatLastServiced) >= VBATINTERVAL) {
            uint32_t vbatTimeDelta = currentTime - vbatLastServiced;
            vbatLastServiced = currentTime;
            updateBattery(vbatTimeDelta);
        }
    }

    if (feature(FEATURE_CURRENT_METER)) {
        int32_t ibatTimeSinceLastServiced = cmp32(currentTime, ibatLastServiced);

        if (ibatTimeSinceLastServiced >= IBATINTERVAL) {
            ibatLastServiced = currentTime;
            updateCurrentMeter(ibatTimeSinceLastServiced, &masterConfig.rxConfig, masterConfig.flight3DConfig.deadband3d_throttle);
        }
    }

    if (ARMING_FLAG(ARMED)) {
        LED0_ON;
    } else {
        if (IS_RC_MODE_ACTIVE(BOXARM) == 0) {
            ENABLE_ARMING_FLAG(OK_TO_ARM);
        }

        if (!STATE(SMALL_ANGLE)) {
            DISABLE_ARMING_FLAG(OK_TO_ARM);
        }

        batteryState_e batState = getBatteryState();
        if ( isCalibrating() || 
             isPreparingDataflash() || 
             isHardwareFailing() ||
             !failsafeIsReceivingRxData() ||
             (feature(FEATURE_VBAT) && (batState == BATTERY_CRITICAL || batState == BATTERY_WARNING || batState == BATTERY_NOT_PRESENT)) ||
             (!sonarIsAvailable() && (IS_RC_MODE_ACTIVE(BOXAHDIRECT) || IS_RC_MODE_ACTIVE(BOXAHASSIST))) ) {

            warningLedFlash();
            DISABLE_ARMING_FLAG(OK_TO_ARM);
        } else {
            if (ARMING_FLAG(OK_TO_ARM)) {
                warningLedDisable();
            } else {
                warningLedFlash();
            }
        }

        if (!ARMING_FLAG(OK_TO_ARM)) {
            shouldAnnounceArmingReady = true;
        }
        else if (ARMING_FLAG(OK_TO_ARM) && shouldAnnounceArmingReady) {
            beeper(BEEPER_READY_TO_ARM);
            shouldAnnounceArmingReady = false;
        }

        warningLedUpdate();
    }

    beeperUpdate();          //call periodic beeper handler

#ifdef TELEMETRY
    telemetryCheckState();
#endif

    handleSerial();

#ifdef GPS
    if (sensors(SENSOR_GPS)) {
        updateGpsIndicator(currentTime);
    }
#endif

    // Read out gyro temperature. can use it for something somewhere. maybe get MCU temperature instead? lots of fun possibilities.
    if (gyro.temperature)
        gyro.temperature(&telemTemperature1);
}

void mwDisarm(void)
{
    if (ARMING_FLAG(ARMED)) {
        DISABLE_ARMING_FLAG(ARMED);
        DISABLE_ARMING_FLAG(OK_TO_ARM);

#ifdef BLACKBOX
        if (feature(FEATURE_BLACKBOX)) {
            finishBlackbox();
        }
#endif

        beeper(BEEPER_DISARM);      // emit disarm tone
    }
}

#define TELEMETRY_FUNCTION_MASK (FUNCTION_TELEMETRY_FRSKY | FUNCTION_TELEMETRY_HOTT | FUNCTION_TELEMETRY_SMARTPORT | FUNCTION_TELEMETRY_LTM | FUNCTION_TELEMETRY_MAVLINK)

void releaseSharedTelemetryPorts(void) {
    serialPort_t *sharedPort = findSharedSerialPort(TELEMETRY_FUNCTION_MASK, FUNCTION_MSP);
    while (sharedPort) {
        mspReleasePortIfAllocated(sharedPort);
        sharedPort = findNextSharedSerialPort(TELEMETRY_FUNCTION_MASK, FUNCTION_MSP);
    }
}

void mwArm(void)
{
    if (ARMING_FLAG(OK_TO_ARM)) {
        if (ARMING_FLAG(ARMED)) {
            return;
        }
        if (IS_RC_MODE_ACTIVE(BOXFAILSAFE)) {
            return;
        }
        if (!ARMING_FLAG(PREVENT_ARMING)) {
            ENABLE_ARMING_FLAG(ARMED);
            headFreeModeHold = DECIDEGREES_TO_DEGREES(attitude.values.yaw);

#ifdef BLACKBOX
            if (feature(FEATURE_BLACKBOX)) {
                serialPort_t *sharedBlackboxAndMspPort = findSharedSerialPort(FUNCTION_BLACKBOX, FUNCTION_MSP);
                if (sharedBlackboxAndMspPort) {
                    mspReleasePortIfAllocated(sharedBlackboxAndMspPort);
                }
                startBlackbox();
            }
#endif
            disarmAt = millis() + masterConfig.auto_disarm_delay * 1000;   // start disarm timeout, will be extended when throttle is nonzero

            //beep to indicate arming
            beeper(BEEPER_ARM);

            return;
        }
    }

    if (!ARMING_FLAG(ARMED)) {
        beeperConfirmationBeeps(1);
    }
}

void updateMagHold(void)
{
    if (ABS(rcCommand[YAW]) < 70 && FLIGHT_MODE(MAG_MODE)) {
        int16_t dif = DECIDEGREES_TO_DEGREES(attitude.values.yaw) - magHold;
        if (dif <= -180)
            dif += 360;
        if (dif >= +180)
            dif -= 360;
        dif *= -masterConfig.yaw_control_direction;
        if (STATE(SMALL_ANGLE))
            rcCommand[YAW] -= dif * currentProfile->pidProfile.P8[PIDMAG] / 30;    // 18 deg
    } else
        magHold = DECIDEGREES_TO_DEGREES(attitude.values.yaw);
}

typedef enum {
#ifdef MAG
    UPDATE_COMPASS_TASK,
#endif
#ifdef BARO
    UPDATE_BARO_TASK,
#endif
#ifdef SONAR
    UPDATE_SONAR_TASK,
#endif
    UPDATE_DISPLAY_TASK
} periodicTasks;

#define PERIODIC_TASK_COUNT (UPDATE_DISPLAY_TASK + 1)


void executePeriodicTasks(void)
{
    static int periodicTaskIndex = 0;

    switch (periodicTaskIndex++) {
#ifdef MAG
    case UPDATE_COMPASS_TASK:
        if (sensors(SENSOR_MAG)) {
            updateCompass(&masterConfig.calibration.magZero);
        }
        break;
#endif

#ifdef BARO
    case UPDATE_BARO_TASK:
        if (sensors(SENSOR_BARO)) {
            baroUpdate(currentTime);
        }
        break;
#endif

#ifdef SONAR
    case UPDATE_SONAR_TASK:
        if (sensors(SENSOR_SONAR)) {
            sonarUpdate();
        }
        break;
#endif
#ifdef DISPLAY
    case UPDATE_DISPLAY_TASK:
        if (feature(FEATURE_DISPLAY)) {
            updateDisplay();
        }
        break;
#endif
    }

    if (periodicTaskIndex >= PERIODIC_TASK_COUNT) {
        periodicTaskIndex = 0;
    }
}

void processRx(void)
{
    static bool armedBeeperOn = false;

    calculateRxChannelsAndUpdateFailsafe(currentTime);

    // in 3D mode, we need to be able to disarm by switch at any time
    if (feature(FEATURE_3D)) {
        if (!IS_RC_MODE_ACTIVE(BOXARM))
            mwDisarm();
    }

    updateRSSI(currentTime);

    if (feature(FEATURE_FAILSAFE)) {

        if (currentTime > FAILSAFE_POWER_ON_DELAY_US && !failsafeIsMonitoring()) {
            failsafeStartMonitoring();
        }

        failsafeUpdateState();
    }

    throttleStatus_e throttleStatus = calculateThrottleStatus(&masterConfig.rxConfig, masterConfig.flight3DConfig.deadband3d_throttle);

	if(throttleStatus == THROTTLE_LOW && feature(FEATURE_MOTOR_STOP)){
		ENABLE_STATE(MOTOR_STOP);
	}
	else {
		DISABLE_STATE(MOTOR_STOP);
	}
	
	
    if (!(ARMING_FLAG(ARMED)) || (throttleStatus == THROTTLE_LOW && feature(FEATURE_MOTOR_STOP)) || (throttleStatus == THROTTLE_LOW && !(IS_RC_MODE_ACTIVE(BOXAIRMODE)))) {
        pidResetErrorGyro();
        ENABLE_STATE(ANTI_WINDUP);
    }
    else {
        DISABLE_STATE(ANTI_WINDUP);
    }

    // When armed and motors aren't spinning, do beeps and then disarm
    // board after delay so users without buzzer won't lose fingers.
    // mixTable constrains motor commands, so checking  throttleStatus is enough
    if (ARMING_FLAG(ARMED)
        && feature(FEATURE_MOTOR_STOP)
        && !STATE(FIXED_WING)
    ) {
        if (throttleStatus == THROTTLE_LOW) {
            if (masterConfig.auto_disarm_delay != 0
                && (int32_t)(disarmAt - millis()) < 0
            ) {
                // auto-disarm configured and delay is over
                mwDisarm();
                armedBeeperOn = false;
            } else {
                // still armed; do warning beeps while armed
                beeper(BEEPER_ARMED);
                armedBeeperOn = true;
            }
        } else {
            // throttle is not low
            if (masterConfig.auto_disarm_delay != 0) {
                // extend disarm time
                disarmAt = millis() + masterConfig.auto_disarm_delay * 1000;
            }

            if (armedBeeperOn) {
                beeperSilence();
                armedBeeperOn = false;
            }
        }
    }

    // Arming happens here, but according to BOXARM box mode which is only set later
    processRcStickPositions(&masterConfig.rxConfig, throttleStatus, masterConfig.disarm_kill_switch);

    // Sets box modes (including BOXARM, thus will arm next RX cycle giving annexCode time to update OK_TO_ARM flag)
    updateActivatedModes(currentProfile->modeActivationConditions);

    if (!cliMode) {
        updateAdjustmentStates(currentProfile->adjustmentRanges);
        processRcAdjustments(currentControlRateProfile, &masterConfig.rxConfig);
    }

    bool canUseHorizonMode = true;

    if ((IS_RC_MODE_ACTIVE(BOXANGLE) || (feature(FEATURE_FAILSAFE) && failsafeIsActive()) || IS_RC_MODE_ACTIVE(BOXSNL)) && (sensors(SENSOR_ACC))) {
        // bumpless transfer to Level mode
        canUseHorizonMode = false;

        if (!FLIGHT_MODE(ANGLE_MODE)) {
            ENABLE_FLIGHT_MODE(ANGLE_MODE);
        }
    } else {
        DISABLE_FLIGHT_MODE(ANGLE_MODE); // failsafe support
    }

    if (IS_RC_MODE_ACTIVE(BOXHORIZON) && canUseHorizonMode) {

        DISABLE_FLIGHT_MODE(ANGLE_MODE);

        if (!FLIGHT_MODE(HORIZON_MODE)) {
            ENABLE_FLIGHT_MODE(HORIZON_MODE);
        }
    } else {
        DISABLE_FLIGHT_MODE(HORIZON_MODE);
    }

    if (FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE)) {
        LED1_ON;
    } else {
        LED1_OFF;
    }

#ifdef  MAG
    if (sensors(SENSOR_ACC) || sensors(SENSOR_MAG)) {
        if (IS_RC_MODE_ACTIVE(BOXMAG)) {
            if (!FLIGHT_MODE(MAG_MODE)) {
                ENABLE_FLIGHT_MODE(MAG_MODE);
                magHold = DECIDEGREES_TO_DEGREES(attitude.values.yaw);
            }
        } else {
            DISABLE_FLIGHT_MODE(MAG_MODE);
        }
        if (IS_RC_MODE_ACTIVE(BOXHEADFREE)) {
            if (!FLIGHT_MODE(HEADFREE_MODE)) {
                ENABLE_FLIGHT_MODE(HEADFREE_MODE);
            }
        } else {
            DISABLE_FLIGHT_MODE(HEADFREE_MODE);
        }
        if (IS_RC_MODE_ACTIVE(BOXHEADADJ)) {
            headFreeModeHold = DECIDEGREES_TO_DEGREES(attitude.values.yaw); // acquire new heading
        }
    }
#endif

#ifdef GPS
    if (sensors(SENSOR_GPS)) {
        updateGpsWaypointsAndMode();
    }
#endif

    if (IS_RC_MODE_ACTIVE(BOXPASSTHRU)) {
        ENABLE_FLIGHT_MODE(PASSTHRU_MODE);
    } else {
        DISABLE_FLIGHT_MODE(PASSTHRU_MODE);
    }

    if (masterConfig.mixerMode == MIXER_FLYING_WING || masterConfig.mixerMode == MIXER_AIRPLANE) {
        DISABLE_FLIGHT_MODE(HEADFREE_MODE);
    }

#ifdef TELEMETRY
    if (feature(FEATURE_TELEMETRY)) {
        if ((!masterConfig.telemetryConfig.telemetry_switch && ARMING_FLAG(ARMED)) ||
                (masterConfig.telemetryConfig.telemetry_switch && IS_RC_MODE_ACTIVE(BOXTELEMETRY))) {

            releaseSharedTelemetryPorts();
        } else {
            // the telemetry state must be checked immediately so that shared serial ports are released.
            telemetryCheckState();
            mspAllocateSerialPorts(&masterConfig.serialConfig);
        }
    }
#endif

}

// Function for loop trigger
bool shouldRunLoop(uint32_t loopTime) {
	bool loopTrigger = false;

    if (masterConfig.gyroSync) {
        if (gyroSyncCheckUpdate() || (int32_t)(currentTime - (loopTime + GYRO_WATCHDOG_DELAY)) >= 0) {
            loopTrigger = true;
        }
    } else if (masterConfig.looptime == 0 || (int32_t)(currentTime - loopTime) >= 0){
        loopTrigger = true;
    }

    return loopTrigger;
}

void filterRc(void){
    static int16_t lastCommand[4] = { 0, 0, 0, 0 };
    static int16_t deltaRC[4] = { 0, 0, 0, 0 };
    static int16_t factor, rcInterpolationFactor;
    static biquad_t filteredCycleTimeState;
    static bool filterInitialised;
    uint16_t filteredCycleTime;
    uint16_t rxRefreshRate;

    // Set RC refresh rate for sampling and channels to filter
    initRxRefreshRate(&rxRefreshRate);

    // Calculate average cycle time (1Hz LPF on cycle time)
    if (!filterInitialised) {
        filterInitBiQuad(1, &filteredCycleTimeState, 0);
        filterInitialised = true;
    }

    filteredCycleTime = filterApplyBiQuad((float) cycleTime, &filteredCycleTimeState);

    rcInterpolationFactor = rxRefreshRate / filteredCycleTime + 1;

    if (isRXDataNew) {
        for (int channel=0; channel < 4; channel++) {
            deltaRC[channel] = rcCommand[channel] -  (lastCommand[channel] - deltaRC[channel] * factor / rcInterpolationFactor);
            lastCommand[channel] = rcCommand[channel];
        }

        factor = rcInterpolationFactor - 1;
    } else {
        factor--;
    }

    // Interpolate steps of rcCommand
    if (factor > 0) {
        for (int channel=0; channel < 4; channel++) {
            rcCommand[channel] = lastCommand[channel] - deltaRC[channel] * factor/rcInterpolationFactor;
         }
    } else {
        factor = 0;
    }
}

void loop(void)
{
    static uint32_t loopTime;
    static bool delayedCalibrationDone = false;

    if (!delayedCalibrationDone && currentTime >= 2000000) {
        // Start gyro calibration
        gyroSetCalibrationCycles(CALIBRATING_GYRO_CYCLES);

        // Indicate calibration done
        delayedCalibrationDone = true;
    }

    updateRx(currentTime);

    if (shouldProcessRx(currentTime)) {
        processRx();
        isRXDataNew = true;
    } else {
        
        g_altitudeHoldLog.debugTelemetyValues[0] = debug[0] ;
        g_altitudeHoldLog.debugTelemetyValues[1] = debug[1] ;
        g_altitudeHoldLog.debugTelemetyValues[2] = debug[2] ;
        g_altitudeHoldLog.debugTelemetyValues[3] = debug[3] ;

        // not processing rx this iteration
        executePeriodicTasks();

        // if GPS feature is enabled, gpsThread() will be called at some intervals to check for stuck
        // hardware, wrong baud rates, init GPS if needed, etc. Don't use SENSOR_GPS here as gpsThread() can and will
        // change this based on available hardware
#ifdef GPS
        if (feature(FEATURE_GPS)) {
            gpsThread();
        }
#endif
    }

    currentTime = micros();
    if (shouldRunLoop(loopTime)) {
        loopTime = currentTime + targetLooptime;

        imuUpdate();

       
        // Measure loop rate just after reading the sensors
        currentTime = micros();
        cycleTime = (int32_t)(currentTime - previousTime);
        previousTime = currentTime;

        dT = (float)cycleTime * 0.000001f;

        annexCode();

        if (masterConfig.rxConfig.rcSmoothing) {
            filterRc();
        }

        // AMIMON: Update mode states for altitude hold and direction
		
		
		
        // Some code to be processed on each RX update _after_ annexCode and _before_ the code that will change rcCommand
        if (isRXDataNew) {
            
			if(sensors(SENSOR_BARO)) {
             initSnlState();
			}
            
#if defined(BARO) || defined(SONAR)
            if ( !FLIGHT_MODE(SNL_MODE) && (sensors(SENSOR_BARO) || sensors(SENSOR_SONAR))) {
                updateAltHoldState(currentTime);
            }
#endif
            updateDirectionModeState();

           
        }

        isRXDataNew = false;

        // AMIMON: Update altitude estimate
#if defined(BARO) || defined(SONAR)
//        setSonarAllowanceFlag(SONAR_ALLOWED_SNL_EXCEPTION);
        calculateEstimatedAltitude(currentTime);
#endif

        // AMIMON: Update forward velocity estimate for direction mode
        update1AxisVelocityEstimate();
        update2AxisVelocityEstimate();

#ifdef MAG
        if (sensors(SENSOR_MAG)) {
            updateMagHold();
        }
#endif

#ifdef GTUNE
        updateGtuneState();
#endif

        // AMIMON: Apply PANIC mode
        if (FLIGHT_MODE(SNL_MODE)) {
            applySnlMode(&currentProfile->pidProfile, &masterConfig.airplaneConfig, currentTime);
        }
        else {
            // AMIMON: Apply direction mode
            // Direction mode is only applicable to ANGLE mode and has no meaning in RATE mode
            if (FLIGHT_MODE(DIRECTION_MODE) && FLIGHT_MODE(ANGLE_MODE)) {
                applyDirectionModeController(currentControlRateProfile);
            }

#ifdef GPS
            if (sensors(SENSOR_GPS)) {
                if ((FLIGHT_MODE(GPS_HOME_MODE) || FLIGHT_MODE(GPS_HOLD_MODE)) && STATE(GPS_FIX_HOME)) {
                    updateGpsStateForHomeAndHoldMode();
                }
            }
#endif
        }

        // AMIMON: Apply altitude hold
#if defined(BARO) || defined(SONAR)
        if (sensors(SENSOR_BARO) && sensors(SENSOR_SONAR)) {
            if ((FLIGHT_MODE(ALTHOLD_DIRECT) || FLIGHT_MODE(ALTHOLD_ASSIST)) && FLIGHT_MODE(ANGLE_MODE)) {
                throttleStatus_e throttleStatus = calculateThrottleStatus(&masterConfig.rxConfig, masterConfig.flight3DConfig.deadband3d_throttle);

                applyAltHold(&masterConfig.airplaneConfig, throttleStatus, currentTime);
            }
        }
#endif

        // If we're armed, at minimum throttle, and we do arming via the
        // sticks, do not process yaw input from the rx.  We do this so the
        // motors do not spin up while we are trying to arm or disarm.
        // Allow yaw control for tricopters if the user wants the servo to move even when unarmed.
        if (isUsingSticksForArming() && rcData[THROTTLE] <= masterConfig.rxConfig.mincheck
#ifndef USE_QUAD_MIXER_ONLY
                && !((masterConfig.mixerMode == MIXER_TRI || masterConfig.mixerMode == MIXER_CUSTOM_TRI) && masterConfig.mixerConfig.tri_unarmed_servo)
                && masterConfig.mixerMode != MIXER_AIRPLANE
                && masterConfig.mixerMode != MIXER_FLYING_WING
#endif
        ) {
            rcCommand[YAW] = 0;
        }

        // Apply throttle tilt compensation
        if (!STATE(FIXED_WING)) {
            if (currentProfile->throttle_tilt_compensation_strength && FLIGHT_MODE(ANGLE_MODE)) {
                rcCommand[THROTTLE] = masterConfig.escAndServoConfig.minthrottle
                                      + (rcCommand[THROTTLE] - masterConfig.escAndServoConfig.minthrottle) * calculateThrottleTiltCompensationFactor(currentProfile->throttle_tilt_compensation_strength);
            }
        }

#if defined(BARO) || defined(SONAR)
        if (!((sensors(SENSOR_BARO) || sensors(SENSOR_SONAR)) && (FLIGHT_MODE(ALTHOLD_DIRECT) || FLIGHT_MODE(ALTHOLD_ASSIST)))) {
            pidResetCustomInclinationLimits();
        }
#endif

        // PID - note this is function pointer set by setPIDController()
        pidController(
            &currentProfile->pidProfile,
            currentControlRateProfile,
            &masterConfig.rxConfig
        );

        // Crash detection should happen after PID controller processing - gyro rates and rcCommand are processed there
		
		  /*crash detection*/
        if (sensors(SENSOR_ACC)){
        //updateCrashDetectionAcceleration(currentTime);
		updateOnGroundDetectionAcceleration(currentTime);
        }
		
		//updateCrashDetection(currentTime);
		 updateGroundImpact(currentTime);
		
		/* update on ground flag */
		updateOnGroundDetection(currentTime);

        mixTable();

#ifdef USE_SERVOS
        filterServos();
        writeServos();
#endif

        if (motorControlEnable) {
            writeMotors();
        }

#ifdef BLACKBOX
        if (!cliMode && feature(FEATURE_BLACKBOX)) {
            handleBlackbox();
        }
#endif
    }

#ifdef TELEMETRY
    if (!cliMode && feature(FEATURE_TELEMETRY)) {
        telemetryProcess(&masterConfig.rxConfig, masterConfig.flight3DConfig.deadband3d_throttle);
    }
#endif

#ifdef LED_STRIP
    if (feature(FEATURE_LED_STRIP)) {
        updateLedStrip();
    }
#endif

#ifdef DEBUG_THROTTLE
        debug[0] = STATE(TAKE_OFF) ? 1 : 0;
        debug[1] = rcCommand[THROTTLE];
        debug[2] = rcStickCommand[THROTTLE];
        debug[3] = motor[3];
#endif
}
