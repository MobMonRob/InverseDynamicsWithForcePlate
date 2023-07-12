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
            viconDataAcquisition.waitForFrame();
            ForcePlateDataFrame dataFrame = viconDataAcquisition.grabForcePlataDataFrame(amti);
            for (const ForcePlateData &dataPoint : dataFrame.forcePlateDataVector)
            {
                std::cout << dataFrame.frameNumber << ", " << amti << ", "
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
