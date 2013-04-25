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
