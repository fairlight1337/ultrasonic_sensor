#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <string>
#include <sstream>
#include <SerialStream.h>

using namespace std;
using namespace LibSerial;


int main(int argc, char **argv) {
  ros::init(argc, argv, "ultrasonic_sensor_mb1033");
  ros::NodeHandle nh("~");
  
  ros::Publisher pubDistance = nh.advertise<std_msgs::Int32>("distance", 1);
  
  string strSerialPort = "/dev/ttyUSB0";
  SerialPort serialPort(strSerialPort.c_str());
  serialPort.Open(SerialPort::BAUD_9600,
		    SerialPort::CHAR_SIZE_8,
		    SerialPort::PARITY_NONE,
		    SerialPort::STOP_BITS_1,
		    SerialPort::FLOW_CONTROL_HARD);
  
  if(serialPort.IsOpen()) {
    while(ros::ok()) {
      string strLine = serialPort.ReadLine(0, '\r');
      
      if(strLine.length() == 6) {
	string strDistance = strLine.substr(1, 4);
	istringstream issBuffer(strDistance);
	
	int nDistance;
	issBuffer >> nDistance;
	
	std_msgs::Int32 int32Out;
	int32Out.data = nDistance;
	
	pubDistance.publish(int32Out);
      }
      
      ros::spinOnce();
    }
    
    serialPort.Close();
    ros::shutdown();
    
    return 0;
  } else {
    ROS_ERROR("Couldn't open serial terminal %s, quitting.", strSerialPort.c_str());
    return -1;
  }
}
