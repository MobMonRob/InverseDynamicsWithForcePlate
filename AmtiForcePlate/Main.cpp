#include "DataStreamClient.h"

#include <iostream>
#include "ForcePlateDataAcquisition.hpp"

using namespace Acquisition;

int main()
{
    try
    {
        ForcePlateDataAcquisition forcePlateDataAcquisition(ForcePlateDataAcquisition::create());

        const std::string &amti = ForcePlateDataAcquisition::amti2;

        while (true)
        {
            long frameNumber = forcePlateDataAcquisition.waitForFrame();
            std::vector<ForcePlateData> dataVector = forcePlateDataAcquisition.grabForcePlataDataFrame(amti);
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
