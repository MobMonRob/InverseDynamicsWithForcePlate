// ROS standard includes
#include "ros/ros.h"
#include <sstream>

#include <iostream>
#include <limits>

#include "vicon_data_publisher/Force_plate_data.h"
#include "vicon_data_acquisition/ForcePlateDataAcquisition.hpp"

using namespace vicon_data_publisher;
using namespace Acquisition;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Force_plate_data_publisher");
    ros::NodeHandle n;
    ros::Publisher publisher_Force_plate_data = n.advertise<Force_plate_data>("Force_plate_data", 1000);
    ROS_INFO("Force_plate_data_publisher started.");

    ForcePlateDataAcquisition forcePlateDataAcquisition(ForcePlateDataAcquisition::create(ForcePlateDataAcquisition::defaultHostname, ForcePlateDataAcquisition::defaultAMTI));
    const std::vector<ForcePlateData>& forcePlateDataVectorCache(forcePlateDataAcquisition.getForcePlateDataVectorCache());
    const uint subsampleCount = forcePlateDataVectorCache.size();
    Force_plate_data msg;

    uint prevFrameNum = std::numeric_limits<uint>::max() - 1;

    while (ros::ok())
    {
        const uint frameNumber = forcePlateDataAcquisition.getFrame();

        if (frameNumber > prevFrameNum + 1)
        {
            ROS_WARN("Lost frames from %d to inclusive %d!", prevFrameNum + 1, frameNumber - 1);
        }
        prevFrameNum = frameNumber;

        //////////////////////////////////////////////////////////////

        {
            forcePlateDataAcquisition.updateForcePlateDataVectorCache();

            for (uint i = 0; i < subsampleCount; ++i)
            {
                const ForcePlateData &currentOrigin(forcePlateDataVectorCache[i]);
                
                msg.frameNumber = frameNumber;
                msg.subsampleNumber = i;
                msg.fx_N = currentOrigin.fX;
                msg.fy_N = currentOrigin.fY;
                msg.fz_N = currentOrigin.fZ;
                msg.mx_Nm = currentOrigin.mX;
                msg.my_Nm = currentOrigin.mY;
                msg.mz_Nm = currentOrigin.mZ;

                publisher_Force_plate_data.publish(static_cast<const Force_plate_data&>(msg));
            }
        }

        //////////////////////////////////////////////////////////////

        ros::spinOnce();
    }

    return 0;
}
