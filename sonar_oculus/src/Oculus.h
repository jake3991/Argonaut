/******************************************************************************
 * (c) Copyright 2017 Blueprint Subsea.
 * This file is part of Oculus Viewer
 *
 * Oculus Viewer is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Oculus Viewer is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *****************************************************************************/

#pragma once

#include <stdint.h>

// -----------------------------------------------------------------------------
// Collection of data classes provided by JGS
// updated 10/11/15 for ping is etc
// updated 07/12/15 for additional fields

// All structures are single byte packed
#pragma pack(push, 1)

// The test id contained in the oculus header file
#define OCULUS_CHECK_ID 0x4f53

// -----------------------------------------------------------------------------
enum OculusMasterStatusType : uint8_t
{
  oculusMasterStatusSsblBoot,
  oculusMasterStatusSsblRun,
  oculusMasterStatusMainBoot,
  oculusMasterStatusMainRun,
};

// -----------------------------------------------------------------------------
enum OculusPauseReasonType : uint8_t
{
  oculusPauseMagSwitch,
  oculusPauseBootFromMain,
  oculusPauseFlashError,
  oculusPauseJtagLoad,
};

// -----------------------------------------------------------------------------
enum OculusTemperatureStatusType : uint8_t
{
  oculusTempGood,
  oculusTempOverheat,
  oculusTempReserved,
  oculusTempOvermax,
};

// -----------------------------------------------------------------------------
class OculusConstants
{
public:
    static const uint32_t OSS_MAX_BEAMS = 512;
};

// -----------------------------------------------------------------------------
enum OculusDeviceType : uint16_t
{
  deviceTypeUndefined		= 0,
  deviceTypeImagingSonar 	= 1,
};

// -----------------------------------------------------------------------------
enum OculusPartNumberType : uint16_t
{
  partNumberUndefined     = 0,
  partNumberM750d         = 1032,
  partNumberM750d_Fusion  = 1134,
  partNumberM750d_Artemis = 1135,
  partNumberM370          = 1041,
  partNumberM1200d        = 1042,
};

// -----------------------------------------------------------------------------
enum OculusMessageType : uint16_t
{
  messageSimpleFire         = 0x15,
  messagePingResult         = 0x22,
  messageSimplePingResult   = 0x23,
  messageDummy              = 0xff,
};

// -----------------------------------------------------------------------------
enum PingRateType : uint8_t
{
  pingRateNormal  = 0x00, // 10Hz max ping rate
  pingRateHigh    = 0x01, // 15Hz max ping rate
  pingRateHighest = 0x02, // 40Hz max ping rate
  pingRateLow     = 0x03, // 5Hz max ping rate
  pingRateLowest  = 0x04, // 2Hz max ping rate
  pingRateStandby = 0x05, // Disable ping
};

// -----------------------------------------------------------------------------
enum DataSizeType : uint8_t
{
  dataSize8Bit,
  dataSize16Bit,
  dataSize24Bit,
  dataSize32Bit,
};

// -----------------------------------------------------------------------------
struct OculusMessageHeader
{
public:
  uint16_t oculusId;         // Fixed ID 0x4f53
  uint16_t srcDeviceId;      // The device id of the source
  uint16_t dstDeviceId;      // The device id of the destination
  uint16_t msgId;            // Message identifier
  uint16_t msgVersion;
  uint32_t payloadSize;      // The size of the message payload (header not included)
  uint16_t spare2;
};

// -----------------------------------------------------------------------------
struct OculusSimpleFireMessage
{
public:
  OculusMessageHeader head;     // The standard message header

  uint8_t masterMode;           // mode 0 is flexi mode, needs full fire message (not available for third party developers)
                                // mode 1 - Low Frequency Mode (wide aperture, navigation)
                                // mode 2 - High Frequency Mode (narrow aperture, target identification)
  PingRateType pingRate;        // Sets the maximum ping rate.
  uint8_t networkSpeed;         // Used to reduce the network comms speed (useful for high latency shared links)
  uint8_t gammaCorrection;      // 0 and 0xff = gamma correction = 1.0
                                // Set to 127 for gamma correction = 0.5
  uint8_t flags;                // bit 0: 0 = interpret range as percent, 1 = interpret range as meters
                                // bit 1: 0 = 8 bit data, 1 = 16 bit data
                                // bit 2: 0 = wont send gain, 1 = send gain
                                // bit 3: 0 = send full return message, 1 = send simple return message
                                // bit 4: 0 = gain assistance off, 1 = gain assistance on
                                // bit 5: 0 = low power mode off, 1 = low power mode on
  double range;                 // The range demand in percent or meters depending on flags
  double gainPercent;           // The gain demand if gain assistance is off or intensity demand if gain assistance is on
  double speedOfSound;          // meters/second, if set to zero then internal calc will apply using salinity
  double salinity;              // ppt, set to zero if we are in fresh water and 35.0 if we are in salt water
};

// -----------------------------------------------------------------------------
struct OculusSimplePingResult
{
public:
    OculusSimpleFireMessage fireMessage;
    uint32_t pingId; 			/* An incrementing number */
    uint32_t status;
    double frequency;				/* The acoustic frequency (Hz) */
    double temperature;				/* The external temperature (deg C) */
    double pressure;				/* The external pressure (bar) */
    double speedOfSoundUsed;		/* The actual used speed of sound (m/s). May be different to the speed of sound set in the fire message */
    uint32_t pingStartTime;
    DataSizeType dataSize; 			/* The size of the individual data entries */
    double rangeResolution;			/* The range in metres corresponding to a single range line */
    uint16_t nRanges;			/* The number of range lines in the image*/
    uint16_t nBeams;			/* The number of bearings in the image */
    uint32_t imageOffset; 		/* The offset in bytes of the image data from the start of the network message */
    uint32_t imageSize; 		/* The size in bytes of the image data */
    uint32_t messageSize; 		/* The total size in bytes of the network message */
    // *** NOT ADDITIONAL VARIABLES BEYOND THIS POINT ***
    // There will be an array of bearings (shorts) found at the end of the message structure
    // Allocated at run time
    // short bearings[];
    // The bearings to each of the beams in 0.01 degree resolution
};

// -----------------------------------------------------------------------------
struct OculusVersionInfo
{
public:
  uint32_t firmwareVersion0; 	/* The arm0 firmware version major(8 bits), minor(8 bits), build (16 bits) */
  uint32_t firmwareDate0; 		/* The arm0 firmware date */
  uint32_t firmwareVersion1;  	/* The arm1 firmware version major(8 bits), minor(8 bits), build (16 bits) */
  uint32_t firmwareDate1;		/* The arm1 firmware date */
  uint32_t firmwareVersion2;	/* The bitfile version */
  uint32_t firmwareDate2;		/* The bitfile date */
};

// -----------------------------------------------------------------------------
struct OculusStatusMsg
{
public:
  OculusMessageHeader hdr;

  uint32_t   deviceId;
  OculusDeviceType   deviceType;
  OculusPartNumberType partNumber;
  uint32_t   status;
  OculusVersionInfo versinInfo;
  uint32_t   ipAddr;
  uint32_t   ipMask;
  uint32_t   connectedIpAddr;
  uint8_t  macAddr0;
  uint8_t  macAddr1;
  uint8_t  macAddr2;
  uint8_t  macAddr3;
  uint8_t  macAddr4;
  uint8_t  macAddr5;
  double temperature0;
  double temperature1;
  double temperature2;
  double temperature3;
  double temperature4;
  double temperature5;
  double temperature6;
  double temperature7;
  double pressure;
};

#pragma pack(pop)
