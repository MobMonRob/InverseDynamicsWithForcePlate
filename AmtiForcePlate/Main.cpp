#include "DataStreamClient.h"

#include <iostream>
#include "ViconDataAcquisition.hpp"

using namespace Acquisition;

int main()
{
    try
    {
        ViconDataAcquisition viconDataAcquisition(ViconDataAcquisition::create());

        const std::string &amti = ViconDataAcquisition::amti2;

        while (true)
        {
            long frameNumber = viconDataAcquisition.waitForFrame();
            std::vector<ForcePlateData> dataVector = viconDataAcquisition.grabForcePlataDataFrame(amti);
            for (const ForcePlateData &dataPoint : dataVector)
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
