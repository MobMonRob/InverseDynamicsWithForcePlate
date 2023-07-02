// ROS standard includes
#include "ros/ros.h"
#include <sstream>

// Own includes
#include "DataStreamClient.h"

#include <iostream>
#include "ForcePlateDataAcquisition.hpp"

#include "Force_plate_data.h"

using namespace Acquisition;
using namespace ros_communicator;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Force_plate_data_publisher");
  ros::NodeHandle n;
  ros::Publisher publisher = n.advertise<Force_plate_data>("Force_plate_data", 1000);
  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
    Force_plate_data msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
} 
 
