#include "DataStreamClient.h"

#include <iostream>
#include "ForcePlateDataAcquisition.hpp"

using namespace Acquisition;

int main()
{
    try
    {
        const std::string& amti(ForcePlateDataAcquisition::defaultAMTI);
        ForcePlateDataAcquisition forcePlateDataAcquisition(ForcePlateDataAcquisition::create(ForcePlateDataAcquisition::defaultHostname, amti));
        const std::vector<ForcePlateData>& forcePlateDataVectorCache(forcePlateDataAcquisition.getForcePlateDataVectorCache());
        uint subsampleCount = forcePlateDataVectorCache.size();

        while (true)
        {
            const uint frameNumber = forcePlateDataAcquisition.getFrame();
            forcePlateDataAcquisition.updateForcePlateDataVectorCache();
            for (const ForcePlateData &dataPoint : forcePlateDataVectorCache)
            {
                std::cout << frameNumber << ", " << amti << ", "
                          << dataPoint.fX << ", " << dataPoint.fY << ", " << dataPoint.fZ << ", "
                          << dataPoint.mX << ", " << dataPoint.mY << ", " << dataPoint.mZ << std::endl;
            }
        }
    }
    catch (const std::exception &e)
    {
        std::cout << e.what();
    }

    return 0;
}
