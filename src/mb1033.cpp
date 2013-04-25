// Copyright (c) 2013, Jan Winkler <winkler@cs.uni-bremen.de>
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Universität Bremen nor the names of its
//       contributors may be used to endorse or promote products derived from
//       this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <string>
#include <sstream>
#include <SerialStream.h>

using namespace std;
using namespace LibSerial;


int main(int argc, char **argv) {
  // ROS init
  ros::init(argc, argv, "ultrasonic_sensor_mb1033");
  ros::NodeHandle nh("~");
  
  ros::Publisher pubDistance = nh.advertise<std_msgs::Float32>("distance", 1);
  
  // Parameters
  string strSerialPort;
  
  nh.param("device", strSerialPort, string("/dev/ttyUSB0"));
  
  // Serial port init
  SerialPort serialPort(strSerialPort.c_str());

  try {
    serialPort.Open(SerialPort::BAUD_9600,
		    SerialPort::CHAR_SIZE_8,
		    SerialPort::PARITY_NONE,
		    SerialPort::STOP_BITS_1,
		    SerialPort::FLOW_CONTROL_HARD);
  } catch(...) {
    ROS_ERROR("Failed to open device '%s'. Maybe adjust the device path by setting the 'device' parameter accordingly. Quitting.", strSerialPort.c_str());

    ros::shutdown();
    return -1;
  }
  
  // Start operating
  if(serialPort.IsOpen()) {
    while(ros::ok()) {
      string strLine = serialPort.ReadLine(0, '\r');
      
      if(strLine.length() == 6) {
	string strDistance = strLine.substr(1, 4);
	istringstream issBuffer(strDistance);
	
	int nDistance;
	issBuffer >> nDistance;
	
	if(nDistance >= 300 && nDistance <= 5000) { // Distances are at least 300mm and at most 5000mm for this sensor
	  std_msgs::Float32 float32Out;
	  float32Out.data = (float)nDistance / 1000;
	  
	  pubDistance.publish(float32Out);
	}
      }
      
      ros::spinOnce();
    }
    
    serialPort.Close();
    ros::shutdown();
    
    return 0;
  } else {
    ROS_ERROR("Couldn't open serial terminal %s, quitting.", strSerialPort.c_str());
    ros::shutdown();
    return -1;
  }
}
