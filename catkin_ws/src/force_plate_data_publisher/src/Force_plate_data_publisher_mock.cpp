// ROS standard includes
#include "ros/ros.h"
#include <sstream>

#include <iostream>

#include "force_plate_data_publisher/Force_plate_data.h"
#include "force_plate_data_acquisition/ForcePlateDataAcquisition.hpp"

#include <iostream>
#include <random>

using namespace force_plate_data_publisher;
using namespace Acquisition;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Force_plate_data_publisher_mock");
    ros::NodeHandle n;
    ros::Publisher publisher = n.advertise<Force_plate_data>("Force_plate_data", 1000);

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        // obtain a random number from hardware
        std::random_device rd;

        // seed the generator
        std::mt19937 gen(rd());

        // define the range
        std::uniform_int_distribution<> distr(1, 360);

        for (int i = 0; i < 8; i++)
        {
            Force_plate_data msg;

            msg.fx_N = distr(gen);
            msg.fy_N = distr(gen);
            msg.fz_N = distr(gen);
            msg.mx_Nm = distr(gen);
            msg.my_Nm = distr(gen);
            msg.mz_Nm = distr(gen);

            publisher.publish(msg);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
