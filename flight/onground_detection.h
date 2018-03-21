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


 typedef enum {
    ONGROUND = 1,
    UNDETERMINED_ONGROUND,
    NOT_ONGROUND,
} onGroundState_e;


void updateOnGroundDetection(uint32_t currentTime);
void updateOnGroundDetectionRates(float rpyActualRates[3],uint32_t currentTime);
void updateOnGroundRatesAreStable (float rpyActualRates[3],uint32_t currentTime);
void updateOnGroundDetectionAcceleration(uint32_t currentTime);
void switchOnGroundDetectionState(onGroundState_e newState);
void resetOnGroundDetection(void);

extern onGroundState_e g_onGroundState;
