// ROS standard includes
#include "ros/ros.h"
#include <sstream>

#include <iostream>
#include <limits>

#include "force_plate_data_publisher/Force_plate_data.h"
#include "force_plate_data_acquisition/ForcePlateDataAcquisition.hpp"

using namespace force_plate_data_publisher;
using namespace Acquisition;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Force_plate_data_publisher");
    ros::NodeHandle n;
    ros::Publisher publisher = n.advertise<Force_plate_data>("Force_plate_data", 1000);

    ForcePlateDataAcquisition forcePlateDataAcquisition;

    long prevFrameNum = std::numeric_limits<long>::max() - 1;
    while (ros::ok())
    {
        ForcePlateDataFrame dataFrame = forcePlateDataAcquisition.grabDirect(ForcePlateDataAcquisition::amti1);

        if (dataFrame.frameNumber > prevFrameNum + 1)
        {
            ROS_WARN("Lost frames from %ld to inclusive %ld!", prevFrameNum + 1, dataFrame.frameNumber - 1);
        }
        prevFrameNum = dataFrame.frameNumber;

        for (const ForcePlateData &dataPoint : dataFrame.forcePlateDataVector)
        {
            Force_plate_data msg;

            msg.f_x = dataPoint.fX;
            msg.f_y = dataPoint.fY;
            msg.f_z = dataPoint.fZ;
            msg.m_x = dataPoint.mX;
            msg.m_y = dataPoint.mY;
            msg.m_z = dataPoint.mZ;

            publisher.publish(msg);
        }

        ros::spinOnce();
    }

    return 0;
}
