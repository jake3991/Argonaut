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
#include <sys/types.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <string.h>
#include <netdb.h>
#include <stdio.h>
#include <string>
#include <iostream>
#include <arpa/inet.h>
#include <sys/ioctl.h>

#include "Oculus.h"
#include "OculusClient.h"


// ============================================================================
// OsBufferEntry - contains a return message and an embedded image
OsBufferEntry::OsBufferEntry()
{
  m_pImage  = nullptr;
  m_pBrgs   = nullptr;

  m_pRaw    = nullptr;
  m_rawSize = 0;

  memset(&m_rfm, 0, sizeof(OculusSimplePingResult));
}

OsBufferEntry::~OsBufferEntry()
{
  if (m_pImage)
    delete m_pImage;

  m_pImage = nullptr;

  if (m_pBrgs)
    delete m_pBrgs;

  m_pBrgs = nullptr;

  if (m_pRaw)
    delete m_pRaw;

  m_pRaw = nullptr;

  m_rawSize = 0;
}


// ----------------------------------------------------------------------------
// Process a complete payload
void OsBufferEntry::AddRawToEntry(char* pData, uint64_t nData)
{
  // Lock the buffer entry
  m_mutex.lock();

  // Copy the raw image (for logging)
  m_pRaw = (uint8_t*) realloc (m_pRaw, nData);
  if (m_pRaw)
  {
    memcpy(m_pRaw, pData, nData);
    m_rawSize = nData;
  }

  m_mutex.unlock();
}

// ----------------------------------------------------------------------------
// Process the raw data record into image data
void OsBufferEntry::ProcessRaw(char* pData)
{
  m_mutex.lock();

  // Copy the fire message

    OculusMessageHeader head;
    memcpy(&head, pData, sizeof(OculusMessageHeader));
    // Test the image size against the message size
    switch (head.msgId)
    {
        case messageSimplePingResult :
        {
          memcpy(&m_rfm, pData, sizeof(OculusSimplePingResult));
          if (m_rfm.fireMessage.head.payloadSize + sizeof(OculusMessageHeader) == m_rfm.imageOffset + m_rfm.imageSize)
          {
            // Should be safe to copy the image
            m_pImage = (unsigned char*) realloc(m_pImage,  m_rfm.imageSize);

            if (m_pImage)
              memcpy(m_pImage, pData + m_rfm.imageOffset, m_rfm.imageSize);

            // Copy the bearing table
            m_pBrgs = (short*) realloc(m_pBrgs, m_rfm.nBeams * sizeof(short));

            if (m_pBrgs)
              memcpy(m_pBrgs, pData + sizeof(OculusSimplePingResult), m_rfm.nBeams * sizeof(short));
          }
          else
            std::cerr << "Error in Simple Return Fire Message. Byte Match:" + std::to_string(m_rfm.fireMessage.head.payloadSize + sizeof(OculusMessageHeader)) + " != " + std::to_string(m_rfm.imageOffset + m_rfm.imageSize);
        } break;
  }
  m_mutex.unlock();
}

// ============================================================================
// OsReadThread - a worker thread used to read OS rfm data from the network
OsReadThread::OsReadThread()
{
  m_pClient   = nullptr;
  m_active    = false;
  m_pToSend   = nullptr;
  m_nToSend   = 0;
  m_osInject  = 0;
  m_nFlushes  = 0;
  m_pRxBuffer = nullptr;
  m_nRxIn     = 0;
  m_nRxMax    = 0;
  m_pSocket   = nullptr;
}

OsReadThread::~OsReadThread()
{
  if (m_pRxBuffer)
    delete m_pRxBuffer;

  m_pRxBuffer = nullptr;
  m_nRxIn     = 0;
  m_nRxMax    = 0;
}

// ----------------------------------------------------------------------------
// Thread safe test for activity
bool OsReadThread::IsActive()
{
  bool active = false;

  m_mutex.lock();
  active = m_active;
  m_mutex.unlock();

  return active;
}

// ----------------------------------------------------------------------------
// Thread safe setting of activity
void OsReadThread::SetActive(bool active)
{
  m_mutex.lock();
  m_active = active;
  m_mutex.unlock();
}

// ----------------------------------------------------------------------------
// Start the thread running
void OsReadThread::Startup()
{
  if (IsActive())
    std::cerr << "Cannot start read thread: Already running";
  else
  {
    SetActive(true);
    //start(); // needed for Qthread, but not std::thread
  }
}

// ----------------------------------------------------------------------------
// If we are running then switch off the running flag and wait for the thread to exit
void OsReadThread::Shutdown()
{
  if (IsActive())
  {
    SetActive(false);

    // need a wait condition here
    //wait(500); //??
  }
  else
    std::cerr << "Cannot shut down read thread: Not running";
}


// ----------------------------------------------------------------------------
// Process the contents of the rx buffer
void OsReadThread::ProcessRxBuffer()
{
  int64_t pktSize = (int64_t)sizeof(OculusMessageHeader);

  // Check for header message in rx buffer
  if (m_nRxIn >= (int64_t)sizeof(OculusMessageHeader))
  {
    // Read the message
    OculusMessageHeader* pOmh = (OculusMessageHeader*) m_pRxBuffer;

    // Invalid data in the header - flush the buffer
    // It might be possible to try and find a vlid header by searching for the correct id here
    if (pOmh->oculusId != 0x4f53)
    {
      m_nFlushes++;
      std::cerr << "Having to flush buffer, unrecognised data. #:" + std::to_string(m_nFlushes);
      m_nRxIn = 0;
      return;
    }

    pktSize += pOmh->payloadSize;

    // If we have the payload the process the data
    if (m_nRxIn >= pktSize)
    {
      ProcessPayload(m_pRxBuffer, pktSize);

      // If there is any additional data in the buffer shift it
      memmove(m_pRxBuffer, &m_pRxBuffer[pktSize], m_nRxIn - pktSize);
      m_nRxIn -= pktSize;
    }
  }
}

// ----------------------------------------------------------------------------
// Process a complete payload
void OsReadThread::ProcessPayload(char* pData, uint64_t nData)
{
  // Cast and test the message
  OculusMessageHeader* pOmh = (OculusMessageHeader*) pData;

  // We are only interested in message ping results
  if (pOmh->msgId == messageSimplePingResult)
  { 
    // Get the next available protected buffer
    OsBufferEntry* pBuffer = &m_osBuffer[m_osInject];
    m_osInject = (m_osInject + 1) % OS_BUFFER_SIZE;

    pBuffer->AddRawToEntry(pData, nData);
    pBuffer->ProcessRaw(pData);

    //emit NewReturnFire(pBuffer);    
  }
  else if (pOmh->msgId != messageDummy )
    std::cerr << "Unrecognised message ID:" + std::to_string(pOmh->msgId) + "\n";
}

// ----------------------------------------------------------------------------
// This is the main read loop
void OsReadThread::run()
{
  unsigned nSent = 0;

  // Cannot progress without a client
  if (!m_pClient)
    return;

/* Do not want run to open sockets

  // Try and open the socket

  m_pSocket = new QTcpSocket;
  m_pSocket->connectToHost(m_hostname, m_port);

  if (!m_pSocket->waitForConnected(3000))
  {
    QString error = "Connection failed for: " + m_hostname + " :" + std::to_string(m_port) + " Reason:" + m_pSocket->errorString();

    SetActive(false);
    //emit NotifyConnectionFailed(error);

    delete m_pSocket;
    m_pSocket = nullptr;

    return;
  }

  m_pSocket->setSocketOption(QAbstractSocket::LowDelayOption, 1);
  // Brought through from John's C# code
  m_pSocket->setSocketOption(QAbstractSocket::KeepAliveOption, true);
  m_pSocket->setReadBufferSize(200000);
*/

  if (IsActive())
  {
    // Send any waiting transmit data
    m_sending.lock();
    {
      if (m_pToSend && m_nToSend > 0)
      {
        nSent++;
        nSent = write(*m_pSocket, m_pToSend, m_nToSend);

        delete m_pToSend;
        m_pToSend = nullptr;
        m_nToSend = 0;
      }
    }
    m_sending.unlock();

    // Check for any data in the rx buffer
    int64_t bytesAvailable;
	  ioctl(*m_pSocket, FIONREAD, &bytesAvailable);

    if (bytesAvailable > 0)
    { 
      // bytesAvailable could be larger than INT_MAX
      if (bytesAvailable > 1000000)
        return;

      // Make sure there is enough room in the buffer - expand if required
      if (m_nRxIn + bytesAvailable > m_nRxMax)
      {
        m_nRxMax = m_nRxIn + bytesAvailable;
        m_pRxBuffer = (char*) realloc (m_pRxBuffer, m_nRxMax);
      }

      // Read the new data into the buffer at the inject point
      unsigned bytesRead = read(*m_pSocket, (char*)&m_pRxBuffer[m_nRxIn], bytesAvailable);

      m_nRxIn += bytesRead;

      // Test the Rx Buffer for new messages
      ProcessRxBuffer();

    }
    
    //m_pSocket->waitForReadyRead(5);
  }


// See above... no port handling in class
  //m_pSocket->disconnectFromHost();
  //m_pSocket->abort();
  // m_pSocket->close();

  //delete m_pSocket;
  //m_pSocket = nullptr;

  //std::cerr << "Read Thread exited";
}


// ============================================================================
// OsClientCtrl - used to communicate with the oculus sonar

OsClientCtrl::OsClientCtrl()
{
  m_hostname   = "localhost";
  m_mask       = "";

  // Link back this client to the reader thread
  m_readData.m_pClient = this;
}

OsClientCtrl::~OsClientCtrl()
{
  m_readData.Shutdown(); 
}

// ----------------------------------------------------------------------------
// Attempt to connect to the current host and port number
bool OsClientCtrl::Connect()
{
  //m_readData.setObjectName("Read Thread");
  m_readData.m_hostname   = m_hostname;
  m_readData.m_port       = 52100;
  m_readData.Startup();

  return true;
}

// ----------------------------------------------------------------------------
// If the port is open then disconnect
bool OsClientCtrl::Disconnect()
{
  m_readData.Shutdown();

  m_mask = "";

  return true;
}

// ----------------------------------------------------------------------------
// Pass through the underlying socket status
bool OsClientCtrl::IsOpen()
{
  return (m_readData.IsActive());
}

// ----------------------------------------------------------------------------
// Write the data to the socket
// This function adds data into the send buffer which is then sent by the read
// thread - this is to make sure all socket access is within the same thread.
void OsClientCtrl::WriteData(char* pData, uint16_t length)
{
  m_readData.m_sending.lock();

  if (m_readData.m_nToSend == 0)
  {
    m_readData.m_nToSend = length;
    m_readData.m_pToSend = (char*) realloc (m_readData.m_pToSend, length);
    memcpy(m_readData.m_pToSend, pData, length);
  }

  m_readData.m_sending.unlock();
}

// ----------------------------------------------------------------------------
// Fire the oculus sonar using the simple fire message
void OsClientCtrl::Fire(int mode, int pingRate, double range, double gain, double speedOfSound, double salinity)
{
  if (IsOpen())
  {
    OculusSimpleFireMessage sfm;
    memset(&sfm, 0, sizeof(OculusSimpleFireMessage));

    sfm.head.msgId       = messageSimpleFire;
    sfm.head.srcDeviceId = 0;
    sfm.head.dstDeviceId = 0;
    sfm.head.oculusId    = 0x4f53;

    // Always allow the range to be set as metres
    uint8_t flags = 0x01; //flagsRangeInMeters;
    if (false)
        flags |= 0x10; //flagsGainAssist;

    flags |= 0x08;

    // ##### Enable 512 beams #####
    flags |= 0x40;
    // ############################

    sfm.flags = flags;                        // Turn on the gain assistance
    sfm.gammaCorrection = 0x7f;
    // sfm.pingRate      = pingRateNormal;
    sfm.pingRate      = static_cast<PingRateType>(pingRate);
    sfm.networkSpeed  = 0xff;
    sfm.masterMode    = mode;
    sfm.range         = range;
    sfm.gainPercent   = gain;

    sfm.speedOfSound = speedOfSound;
    sfm.salinity     = salinity;

    WriteData((char*)&sfm, sizeof(OculusSimpleFireMessage));
  }
}

//
void OsClientCtrl::DummyMessage()
{
    if (IsOpen())
    {
        OculusMessageHeader omh;
        memset(&omh, 0, sizeof(OculusMessageHeader));

        omh.msgId = 0xFF;
        omh.oculusId = 0x4f53;
    }
}
