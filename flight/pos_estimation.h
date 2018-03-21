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
    EST_ALT_QUAL_DR,                /* Flying on baro, ground offset is likely incorrect */
    EST_ALT_QUAL_MID,               /* Sonar has been lost but timeout isn't reached yet, we can somewhat trust AGL */
    EST_ALT_QUAL_HIGH,              /* All good */
} estAltitudeQuality_e;

//typedef enum {
//    SONAR_ALLOWED = 0,
//    SONAR_ALLOWED_SNL_EXCEPTION,    // Allow sonar in angle or for acro/horizon in descent stage
//    SONAR_DISALLOWED,               // Never allow sonar
//} estSonarAllowanceFlag_e;

typedef struct {
    bool        isUpdated;
    uint32_t    lastUpdateTime;
    float       deltaTime;

    float       currentAltitude;
    float       meanAltitude;       /* Average altitude over 15 sample window (only good measurements) */
} estAltitudeBARO_s;

typedef struct {
    bool        isUpdated;
    uint32_t    lastUpdateTime;
    float       deltaTime;

    bool        isOutlier;
    int         sonarOutlierCount;

    float       currentAltitude;
    float       meanAltitude;       /* Average altitude over 5 sample window (only good measurements) */
    float       sonarConsistency;   /* Range in [0;1] showing average percentage of good readings */
    float       sonarRaw;
} estAltitudeSONAR_s;

typedef struct {
    uint32_t    lastUpdateTime;
    float       deltaTime;

    float       correctedAccZ;
    float       biasAccZ;
} estAltitudeIMU_s;

typedef struct {
    float                   aglAltitude;
    float                   aglVelocity;
    float                   baroGroundOffset;
    float                   weightBoost;

    estAltitudeQuality_e    quality;
    uint32_t                qualityChangeTime;
} estAltitudeEST_s;

typedef struct {
    /* Sensor topics */
    estAltitudeBARO_s   baro;
    estAltitudeSONAR_s  sonar;
    estAltitudeIMU_s    imu;

    /* Estimate */
    estAltitudeEST_s    est;
} estAltitude_s;

/* Thresholds between DR / MID / HIGH states */
#define SONAR_CONSISTENCY_LIGHT_THRESHOLD   0.15f       /* Condition for SNL Descent rate */
#define SONAR_CONSISTENCY_LOW_THRESHOLD     0.33f       /* Condition for QUAL_MID */
#define SONAR_CONSISTENCY_HIGH_THRESHOLD    0.75f       /* Condition for QUAL_HIGH */

extern estAltitude_s  g_estAltitude;
extern bool g_resetSnlVelEst;

void resetEstimatedAltitude(void);
void calculateEstimatedAltitude(uint32_t currentTime);
//void setSonarAllowanceFlag(estSonarAllowanceFlag_e newSonarAllowanceFlag);
