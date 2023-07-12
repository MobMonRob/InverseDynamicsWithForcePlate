// ROS standard includes
#include "ros/ros.h"
#include <sstream>

#include <iostream>
#include <limits>

#include "force_plate_data_publisher/Vicon_compound_data.h"
#include "force_plate_data_acquisition/ViconDataAcquisition.hpp"

using namespace force_plate_data_publisher;
using namespace Acquisition;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Force_plate_data_publisher");
    ros::NodeHandle n;
    ros::Publisher publisher = n.advertise<Vicon_compound_data>("Vicon_compound_data", 1000);

    ViconDataAcquisition viconDataAcquisition(ViconDataAcquisition::create());

    long prevFrameNum = std::numeric_limits<long>::max() - 1;
    while (ros::ok())
    {
        Vicon_compound_data msg_Vicon_compound_data;

        viconDataAcquisition.waitForFrame();

        //////////////////////////////////////////////////////////////

        ForcePlateDataFrame dataFrame = viconDataAcquisition.grabForcePlataDataFrame(ViconDataAcquisition::amti2);

        if (dataFrame.frameNumber > prevFrameNum + 1)
        {
            ROS_WARN("Lost frames from %ld to inclusive %ld!", prevFrameNum + 1, dataFrame.frameNumber - 1);
        }
        prevFrameNum = dataFrame.frameNumber;

        for (const ForcePlateData &dataPoint : dataFrame.forcePlateDataVector)
        {
            Force_plate_data msg_Force_plate_data;

            msg_Force_plate_data.f_x = dataPoint.fX;
            msg_Force_plate_data.f_y = dataPoint.fY;
            msg_Force_plate_data.f_z = dataPoint.fZ;
            msg_Force_plate_data.m_x = dataPoint.mX;
            msg_Force_plate_data.m_y = dataPoint.mY;
            msg_Force_plate_data.m_z = dataPoint.mZ;

            msg_Vicon_compound_data.force_plate_data.push_back(std::move(msg_Force_plate_data));
        }

        //////////////////////////////////////////////////////////////

        std::vector<MarkerGlobalTranslationData> markerGlobalTranslationDataVector = viconDataAcquisition.grabMarkerGlobalTranslation();

        for (const MarkerGlobalTranslationData& markerGlobalTranslationData : markerGlobalTranslationDataVector)
        {
            Marker_global_translation msg_Marker_global_translation;

            msg_Marker_global_translation.x = markerGlobalTranslationData.x;
            msg_Marker_global_translation.y = markerGlobalTranslationData.y;
            msg_Marker_global_translation.z = markerGlobalTranslationData.z;
            msg_Marker_global_translation.occluded = markerGlobalTranslationData.occluded;

            msg_Vicon_compound_data.marker_global_translation.push_back(std::move(msg_Marker_global_translation));
        }

        //////////////////////////////////////////////////////////////

        publisher.publish(msg_Vicon_compound_data);

        ros::spinOnce();
    }

    return 0;
}
