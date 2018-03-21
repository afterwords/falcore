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

#define FC_VERSION_FAMILY           14
#define FC_VERSION_MAJOR            4
#define FC_VERSION_MINOR            2

#if BUILDTYPE == ARF
    #define FC_VERSION_BOARD        ARF
#elif BUILDTYPE == RTF
    #define FC_VERSION_BOARD        RTF
#else
    #define FC_VERSION_BOARD        RND
#endif

#define FC_VERSION_BUILD            0

#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)
#define FC_VERSION_STRING STR(FC_VERSION_FAMILY) "." STR(FC_VERSION_BOARD) "." STR(FC_VERSION_MAJOR) "." STR(FC_VERSION_MINOR) "__B" STR(FC_VERSION_BUILD)

#define MW_VERSION              231

extern const char* const targetName;

#define GIT_SHORT_REVISION_LENGTH   7 // lower case hexadecimal digits.
extern const char* const shortGitRevision;

#define BUILD_DATE_LENGTH 11
extern const char* const buildDate;  // "MMM DD YYYY" MMM = Jan/Feb/...

#define BUILD_TIME_LENGTH 8
extern const char* const buildTime;  // "HH:MM:SS"
