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

#include <mutex>
#include <thread>
#include "Oculus.h"
#include "DataWrapper.h"

// ----------------------------------------------------------------------------
// OsBufferEntry - contains a return message and an embedded image
class OsBufferEntry
{
public:
  OsBufferEntry();
  ~OsBufferEntry();

  // Methods
  void AddRawToEntry(char* pData, uint64_t nData);
  void ProcessRaw(char* pData);

  // Data
  OculusSimplePingResult  m_rfm;         // The fixed length return fire message
  unsigned char*          m_pImage;      // The image data
  short*                  m_pBrgs;       // The bearing table
  std::mutex              m_mutex;       // Lock for buffer accesss

  uint8_t*                m_pRaw;        // The raw data
  uint32_t                m_rawSize;     // Size of the raw data record
};


class OsClientCtrl;
#define OS_BUFFER_SIZE 1

// ----------------------------------------------------------------------------
// OsReadThread - a worker thread used to read data from the network for the client
class OsReadThread : public std::thread
{
public:
  OsReadThread();
  ~OsReadThread();

  void run();

  void Startup();
  void Shutdown();
  bool IsActive();
  void SetActive(bool active);
  void ProcessRxBuffer();
  void ProcessPayload(char* pData, uint64_t nData);

  // Data
  OsClientCtrl* m_pClient;   // back pointer to the parent client
  bool          m_active;    // Is the run exec active
  std::mutex    m_mutex;     // Mutex protection for m_active
  std::mutex    m_sending;   // Mutex protection for m_active

  std::string   m_hostname;  // The hostname/address of the sonar
  uint16_t      m_port;      // The port for sonar comms (currently fixed)
  int32_t       m_nFlushes;  // Number of times the rx buffer has had to be flushed

  // The raw receive buffer
  char*         m_pRxBuffer; // The rx buffer for incomming data
  int32_t       m_nRxMax;    // The maximum size of the rx Buffer
  int32_t       m_nRxIn;     // The current amount of unprocessed data in the buffer

  // The recieve buffer for messages
  OsBufferEntry m_osBuffer[OS_BUFFER_SIZE];
  unsigned      m_osInject;   // The position for the next inject

  // The raw send buffer
  int*          m_pSocket;	//point to socket fd
  char*         m_pToSend;
  int64_t       m_nToSend;
};


// ----------------------------------------------------------------------------
// ClientCtrl - used to communicate with the oculus sonar
class OsClientCtrl
{
public:
  OsClientCtrl();
  ~OsClientCtrl();

  bool Connect();
  bool Disconnect();
  bool IsOpen();
  void WriteData(char* pData, uint16_t length);
  void Fire(int mode, int pingRate, double range, double gain, double speedOfSound, double salinity);
  void DummyMessage();

  std::string 	m_hostname;     	// The hostname/address of the sonar
  std::string 	m_mask;
  UserConfig  	m_config;       	// Oculus user configuration
  OsReadThread  m_readData; 		// The worker thread for reading data
};


