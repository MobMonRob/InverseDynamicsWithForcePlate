// ROS standard includes
#include "ros/ros.h"
#include <sstream>

#include <iostream>
#include <limits>
#include <chrono>
using namespace std::chrono;

#include "vicon_data_publisher/Marker_global_translation.h"
#include "vicon_data_acquisition/MarkerDataAcquisition.hpp"

using namespace vicon_data_publisher;
using namespace Acquisition;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Marker_data_publisher");
    ros::NodeHandle n;
    ros::Publisher publisher_Marker_global_translation = n.advertise<Marker_global_translation>("Marker_global_translation", 1000);
    ROS_INFO("Marker_data_publisher started.");

    MarkerDataAcquisition markerDataAcquisition(MarkerDataAcquisition::create(MarkerDataAcquisition::defaultHostname, MarkerDataAcquisition::defaultSubjectName));
    const std::vector<MarkerGlobalTranslationData>& markerGlobalTranslationVectorCache(markerDataAcquisition.getMarkerGlobalTranslationVectorCache());
    const uint markerGlobalTranslationVectorSize = markerGlobalTranslationVectorCache.size();
    Marker_global_translation msg;

    uint prevFrameNum = std::numeric_limits<uint>::max() - 1;

    while (ros::ok())
    {
        auto start = high_resolution_clock::now();
        const uint frameNumber = markerDataAcquisition.getFrame();
        auto stop = high_resolution_clock::now();

        if ((frameNumber % 240) == 0)
        {
            auto duration = duration_cast<microseconds>(stop - start);
            ROS_INFO("getFrame() needed %ld microseconds. (More = better)", duration.count());
        }

        if (frameNumber > prevFrameNum + 1)
        {
            ROS_WARN("Lost frames from %d to inclusive %d!", prevFrameNum + 1, frameNumber - 1);
            auto duration = duration_cast<microseconds>(stop - start);
            ROS_WARN("getFrame() needed %ld microseconds. (More = better)", duration.count());
        }
        prevFrameNum = frameNumber;

        //////////////////////////////////////////////////////////////

        {
            markerDataAcquisition.updateMarkerGlobalTranslationVectorCache();

            for (uint i = 0; i < markerGlobalTranslationVectorSize; ++i)
            {
                const MarkerGlobalTranslationData &currentOrigin(markerGlobalTranslationVectorCache[i]);

                msg.frameNumber = frameNumber;
                msg.markerNumber = i;
                msg.occluded = currentOrigin.occluded;
                msg.x_mm = currentOrigin.x;
                msg.y_mm = currentOrigin.y;
                msg.z_mm = currentOrigin.z;

                publisher_Marker_global_translation.publish(static_cast<const Marker_global_translation&>(msg));
            }
        }

        //////////////////////////////////////////////////////////////

        ros::spinOnce();
    }

    return 0;
}
