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
    ros::Publisher publisher_Force_plate_data = n.advertise<Force_plate_data>("Force_plate_data", 1000);

    ForcePlateDataAcquisition forcePlateDataAcquisition(ForcePlateDataAcquisition::create());

    uint prevFrameNum = std::numeric_limits<uint>::max() - 1;

    while (ros::ok())
    {
        uint frameNumber = forcePlateDataAcquisition.waitForFrame();

        if (frameNumber > prevFrameNum + 1)
        {
            ROS_WARN("Lost frames from %d to inclusive %d!", prevFrameNum + 1, frameNumber - 1);
        }
        prevFrameNum = frameNumber;

        //////////////////////////////////////////////////////////////

        {
            std::vector<ForcePlateData> forcePlateDataVector(forcePlateDataAcquisition.grabForcePlataDataFrame(ForcePlateDataAcquisition::amti2));

            uint8_t forcePlateDataVectorSize = forcePlateDataVector.size();
            for (uint8_t i = 0; i < forcePlateDataVectorSize; ++i)
            {
                const ForcePlateData &currentOrigin(forcePlateDataVector[i]);
                Force_plate_data msg;

                msg.frameNumber = frameNumber;
                msg.subsampleNumber = i;
                msg.fx_N = currentOrigin.fX;
                msg.fy_N = currentOrigin.fY;
                msg.fz_N = currentOrigin.fZ;
                msg.mx_Nm = currentOrigin.mX;
                msg.my_Nm = currentOrigin.mY;
                msg.mz_Nm = currentOrigin.mZ;

                publisher_Force_plate_data.publish(std::move(msg));
            }
        }

        //////////////////////////////////////////////////////////////

        ros::spinOnce();
    }

    return 0;
}
