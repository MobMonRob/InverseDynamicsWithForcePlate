// ROS standard includes
#include "ros/ros.h"
#include <sstream>

#include <iostream>
#include <limits>

#include "force_plate_data_publisher/Force_plate_data.h"
#include "force_plate_data_publisher/Marker_global_translation.h"
#include "force_plate_data_acquisition/ViconDataAcquisition.hpp"

using namespace force_plate_data_publisher;
using namespace Acquisition;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Force_plate_data_publisher");
    ros::NodeHandle n;
    ros::Publisher publisher_Force_plate_data = n.advertise<Force_plate_data>("Force_plate_data", 1000);
    ros::Publisher publisher_Marker_global_translation = n.advertise<Marker_global_translation>("Marker_global_translation", 1000);

    ViconDataAcquisition viconDataAcquisition(ViconDataAcquisition::create());

    uint prevFrameNum = std::numeric_limits<uint>::max() - 1;

    while (ros::ok())
    {
        uint frameNumber = viconDataAcquisition.waitForFrame();

        if (frameNumber > prevFrameNum + 1)
        {
            ROS_WARN("Lost frames from %d to inclusive %d!", prevFrameNum + 1, frameNumber - 1);
        }
        prevFrameNum = frameNumber;

        //////////////////////////////////////////////////////////////

        {
            std::vector<ForcePlateData> forcePlateDataVector(viconDataAcquisition.grabForcePlataDataFrame(ViconDataAcquisition::amti2));

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

        {
            std::vector<MarkerGlobalTranslationData> markerGlobalTranslationDataVector(viconDataAcquisition.grabMarkerGlobalTranslation());

            uint8_t markerGlobalTranslationVectorSize = markerGlobalTranslationDataVector.size();
            for (uint8_t i = 0; i < markerGlobalTranslationVectorSize; ++i)
            {
                const MarkerGlobalTranslationData &currentOrigin(markerGlobalTranslationDataVector[i]);
                Marker_global_translation msg;

                msg.frameNumber = frameNumber;
                msg.markerNumber = i;
                msg.occluded = currentOrigin.occluded;
                msg.x_mm = currentOrigin.x;
                msg.y_mm = currentOrigin.y;
                msg.z_mm = currentOrigin.z;

                publisher_Marker_global_translation.publish(std::move(msg));
            }
        }

        //////////////////////////////////////////////////////////////

        ros::spinOnce();
    }

    return 0;
}
