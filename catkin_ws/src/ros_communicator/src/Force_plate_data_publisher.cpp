// ROS standard includes
#include "ros/ros.h"
#include <sstream>

#include <iostream>

#include "ros_communicator/Force_plate_data.h"

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

    msg.frame_number = 25;

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    publisher.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
} 
 
