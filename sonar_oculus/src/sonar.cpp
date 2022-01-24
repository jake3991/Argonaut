// VSieben@slb.com (original author, 2017)
// pvt@mit.edu     (extensions, 2018)
// jwang92@stevens.edu

#include <algorithm>
#include <arpa/inet.h>
#include <iostream>
#include <math.h>
#include <netdb.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include "Oculus.h"
#include "OculusClient.h"

//opencv
//#include <opencv2/cv.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/highgui/highgui.hpp"

// ROS includes
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sonar_oculus/OculusFire.h>
#include <sonar_oculus/OculusPing.h>

// Dynamic server
#include <dynamic_reconfigure/server.h>
#include <sonar_oculus/OculusParamsConfig.h>

#define BUFLEN 200
#define DATALEN 200000
#define PORT_UDP 52102
#define PORT_TCP 52100

using namespace cv;

// Global sonar configuration
int mode = 1;             // 0 => dev/not used, 1 => ~750khz, 2 => ~1.2Mhz.
int ping_rate = 0;        // 0 => normal
double range = 10;        // m, limited to 120m in mode 1, and 40m in mode 2
double gain = 20;         //%
double soundspeed = 0;    // m/s
double salinity = 0;      // ppm, 0 = freshwater, 35=saltwater

// Error handling function
void error(const char *msg) {
  perror(msg);
  exit(0);
}

// Create sonar oculus control class
OsClientCtrl sonar;
OculusPartNumberType partNumber;

// Callback for dynamic reconfigure server
void callback(sonar_oculus::OculusParamsConfig &config, uint32_t level) {
  mode = config.Mode;
  ping_rate = config.PingRate;
  gain = config.Gain;
  range = config.Range;
  salinity = config.Salinity;

  if (partNumber == OculusPartNumberType::partNumberM750d) {
    if (mode == 1)
      range = std::max(0.1, std::min(range, 120.0));
    else if (mode == 2)
      range = std::max(0.1, std::min(range, 40.0));
  } 
  if (partNumber == OculusPartNumberType::partNumberM1200d) {
    range = std::max(0.1, std::min(range, 30.0));
  } 

  sonar.Fire(mode, ping_rate, range, gain, soundspeed, (double)salinity);
}

// Main program for listening to sonar
int main(int argc, char **argv) {
  // Initialize ROS
  ROS_INFO("Initializing...");
  ros::init(argc, argv, "sonar_oculus");
  ros::NodeHandle nh("~");

  // Read water from /water then set parameter for dynamic_reconfigure 
  std::string water;
  nh.param<std::string>("/water", water, "fresh");
  nh.setParam("/sonar_oculus_node/Salinity", water == "fresh" ? 0 : 35);


  // Setup dynamic server
  dynamic_reconfigure::Server<sonar_oculus::OculusParamsConfig> serverParam;
  dynamic_reconfigure::Server<sonar_oculus::OculusParamsConfig>::CallbackType f;

  f = boost::bind(&callback, _1, _2);
  serverParam.setCallback(f);

  // Variable declarations
  // Communications
  struct sockaddr_in serverUDP, clientUDP;
  struct sockaddr_in serverTCP, clientTCP;
  int sockUDP, sockTCP, sockTCPfd, datagramSize, n;
  int buf_size = DATALEN;
  int keepalive = 1;
  socklen_t lengthServerUDP, lengthClientUDP;
  socklen_t lengthServerTCP, lengthClientTCP;
  char datagramMessage[BUFLEN], buffer[BUFLEN], sonardata[DATALEN];

  // Clear and intialize values of server and client network info
  lengthServerUDP = sizeof(serverUDP);
  bzero((char *)&serverUDP, lengthServerUDP);
  serverUDP.sin_family = AF_INET;
  serverUDP.sin_addr.s_addr = htonl(INADDR_ANY);
  // serverUDP.sin_addr.s_addr = inet_addr(SONAR_ADDR);
  serverUDP.sin_port = htons(PORT_UDP);

  lengthClientUDP = sizeof(clientUDP);
  lengthServerTCP = sizeof(serverTCP);
  
  ROS_INFO("Connecting...");

  std::string model;
  std::string ip;

  nh.getParam("model", model);
  if (!model.empty()) {
    if (model == "M750d") {
      ip = "192.168.2.3";
      partNumber = OculusPartNumberType::partNumberM750d;
    }
    else if (model == "M1200d") {
      ip = "192.168.2.4";
      partNumber = OculusPartNumberType::partNumberM1200d;
    }
    else
      ROS_ERROR_STREAM("Part number not recognized " << model);
  } 

  else {

  while (true) {
    // Create the UDP listening socket or exit
    sockUDP = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockUDP < 0)
      error("Error opening UDP listening socket");
    
    int enable = 1;
    if (setsockopt(sockUDP, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int)) < 0)
      error("setsockopt(SO_REUSEADDR) failed");

    // Bind the UDP socket to address and port, or exit with error
    if (bind(sockUDP, (struct sockaddr *)&serverUDP, lengthServerUDP) < 0)
      error("Error binding UDP listening socket");
    listen(sockUDP, 5);

    int64_t bytesAvailable;
    ioctl(sockUDP, FIONREAD, &bytesAvailable);

    OculusStatusMsg osm;
    if (bytesAvailable > 0) { 
      unsigned bytesRead = read(sockUDP, (char*)&osm, bytesAvailable);

      uint32_t ts = (osm.status >> 14) & 0x0003;
      if (ts == 0) {
        ROS_INFO("Temperature OK; the sonar will ping normally");
      } else if (ts == 1) {
        ROS_WARN("Temperature high; the sonar will ping at reduced rate");
      } else if (ts == 3) {
        ROS_ERROR("Temperature shutdown; the sonar will not longer ping");
        ros::requestShutdown();
        return 0;
      }

      struct in_addr ip_addr;
      ip_addr.s_addr = osm.ipAddr;
      ip = std::string(inet_ntoa(ip_addr));
      partNumber = osm.partNumber;

      if (partNumber == OculusPartNumberType::partNumberM750d)
        model = "M750d";
      else if (partNumber == OculusPartNumberType::partNumberM1200d)
        model = "M1200d";
      else
        ROS_ERROR_STREAM("Part number not recognized " << partNumber);

      close(sockUDP);
      break;
    }

    ros::Duration(1.0).sleep();
  }

  }

  ROS_INFO_STREAM("The IP address is " << ip);
  ROS_INFO_STREAM("Oculus model is " << model);

  ros::Publisher ping_pub = nh.advertise<sonar_oculus::OculusPing>("/sonar_oculus_node/" + model + "/ping", 1);

  bzero((char *)&serverTCP, lengthServerTCP);
  serverTCP.sin_family = AF_INET;
  serverTCP.sin_addr.s_addr = inet_addr(ip.c_str());
  serverTCP.sin_port = htons(PORT_TCP);

  // Create the TCP socket for main communication or exit
  sockTCP = socket(AF_INET, SOCK_STREAM, 0);
  if (sockTCP < 0)
    error("Error opening TCP main socket");
  // Connect to the sonar Server via TCP socket or exit with error
  if (connect(sockTCP, (struct sockaddr *)&serverTCP, lengthServerTCP) < 0)
    error("Error connecting TCP socket");
  if (setsockopt(sockTCP, SOL_SOCKET, SO_RCVBUF, &buf_size, sizeof(buf_size)) <
      0)
    error("Error increasing RCVBUF for TCP socket");
  if (setsockopt(sockTCP, SOL_SOCKET, SO_KEEPALIVE, &keepalive,
                sizeof(keepalive)) < 0)
    error("Error keeping alive option set for TCP socket");
  listen(sockTCP, 5);

  std::string frame_str;

  // Setup Sonar and messages
  // Pass the socket to the control
  sonar.m_readData.m_pSocket = &sockTCP;
  // Connect and instance a thread
  sonar.Connect();

  ROS_INFO("Connected!");

  // Send Ping and initiate data collection
  sonar.Fire(mode, ping_rate, range, gain, soundspeed, salinity);
  // Get frame ID
  if (nh.getParam("frame", frame_str)) {
    ROS_INFO("Got param: %s", frame_str.c_str());
  } else {
    ROS_ERROR("Failed to get param 'frame'");
    frame_str = "/sonar";
  }

  // Run continously
  ros::Rate r(50); // pvt: sonar should be under 40Hz (reduced 100 to 50)
  unsigned int latest_id = 0; // keep track of latest ping to avoid republishing
  while (ros::ok()) {

    // Run the readthread sonar
    sonar.m_readData.run();

    // Get bins and beams #.
    unsigned int nbins = sonar.m_readData.m_osBuffer[0].m_rfm.nRanges;
    unsigned int nbeams = sonar.m_readData.m_osBuffer[0].m_rfm.nBeams;
    unsigned int id = sonar.m_readData.m_osBuffer[0].m_rfm.pingId;

    // Create pointcloud message from sonar data
    if (nbeams > 0 && nbins > 0 && id > latest_id) {
      latest_id = id;

      // sonar image
      if ( sonar.m_readData.m_osBuffer[0].m_rawSize){
        sensor_msgs::Image sonar_image;
        sonar_image.header.stamp = ros::Time::now();
        sonar_image.height = nbins;
        sonar_image.width = nbeams;
        sonar_image.encoding = "8UC1";
        // sonar_image.is_bigendian = 0; // default works
        sonar_image.step = nbeams;
        sonar_image.data.resize(nbeams * nbins);
        std::copy(sonar.m_readData.m_osBuffer[0].m_pImage,
                  sonar.m_readData.m_osBuffer[0].m_pImage +
                  nbins*nbeams,
                  sonar_image.data.begin());

        // fire msg
        sonar_oculus::OculusFire fire_msg;
        fire_msg.header.stamp = ros::Time::now();

        fire_msg.mode = sonar.m_readData.m_osBuffer[0].m_rfm.fireMessage.masterMode;
        fire_msg.gamma =
          sonar.m_readData.m_osBuffer[0].m_rfm.fireMessage.gammaCorrection;
        fire_msg.flags = sonar.m_readData.m_osBuffer[0].m_rfm.fireMessage.flags;
        fire_msg.range = sonar.m_readData.m_osBuffer[0].m_rfm.fireMessage.range;
        fire_msg.gain =
          sonar.m_readData.m_osBuffer[0].m_rfm.fireMessage.gainPercent;
        fire_msg.speed_of_sound =
          sonar.m_readData.m_osBuffer[0].m_rfm.fireMessage.speedOfSound;
        fire_msg.salinity =
          sonar.m_readData.m_osBuffer[0].m_rfm.fireMessage.salinity;

        // sonar ping
        sonar_oculus::OculusPing ping_msg;
        ping_msg.header.frame_id = frame_str;
        ping_msg.header.stamp = fire_msg.header.stamp;

	//compression settings
	std::vector<int> params;
	params.resize(3,0);
	params[0] = cv::IMWRITE_JPEG_QUALITY;
	params[1] = 80; 	

	//define a cv_bridge pointer
	cv_bridge::CvImagePtr cv_ptr;

	//define target format for compression	
	std::stringstream targetFormat;        
	targetFormat << "8UC1";

	//const_cast<sensor_msgs::Image &>(sonar_image).step = 512;
        cv_ptr = cv_bridge::toCvCopy(sonar_image,targetFormat.str());

	//compress image here, note this command compresses AND sets the compressed image to the message
	cv::imencode(".jpg", cv_ptr->image, ping_msg.ping.data, params);
	
	//set compressed image parameters
	ping_msg.ping.format += "; jpeg compressed";

        ping_msg.fire_msg = fire_msg;
        ping_msg.ping_id = id;
        ping_msg.part_number = partNumber;

        ping_msg.start_time = sonar.m_readData.m_osBuffer[0].m_rfm.pingStartTime;
        ping_msg.bearings.resize(nbeams);
        for (int i = 0; i < nbeams; ++i)
            ping_msg.bearings[i] = sonar.m_readData.m_osBuffer[0].m_pBrgs[i];
        ping_msg.range_resolution =
          sonar.m_readData.m_osBuffer[0].m_rfm.rangeResolution;
        ping_msg.num_ranges = nbins;
        ping_msg.num_beams = nbeams;
	

        ping_pub.publish(ping_msg);
      }
    } // if (nbins>0 && nbeams>0 && id>latest_id)

    // Fire sonar (so we sleep while the ping travels)
    r.sleep();
    ros::spinOnce();
  }

  // Disconnect and close
  sonar.Disconnect();

  // Exit
  close(sockTCP);
  return 0;

} // main
