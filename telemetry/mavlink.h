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

#ifndef TELEMETRY_MAVLINK_H_
#define TELEMETRY_MAVLINK_H_

void initMAVLinkTelemetry(telemetryConfig_t *initialTelemetryConfig);
void handleMAVLinkTelemetry(void);
void checkMAVLinkTelemetryState(void);

void freeMAVLinkTelemetryPort(void);
void configureMAVLinkTelemetryPort(void);

#ifdef MAVLINK_DEBUG_MESSAGE
// here you can assign variables to be sent to ProSight OSD
extern int16_t             mav_debug[4];
#endif /* MAVLINK_DEBUG_MESSAGE */

#endif /* TELEMETRY_MSP_H_ */
