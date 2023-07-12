#include "DataStreamClient.h"

#include <iostream>
#include "ForcePlateDataAcquisition.hpp"

using namespace Acquisition;

int main()
{
    try
    {
        ForcePlateDataAcquisition forcePlateDataAcquisition;

        const std::string &amti = ForcePlateDataAcquisition::amti2;

        while (true)
        {
            ForcePlateDataFrame dataFrame = forcePlateDataAcquisition.grabDirect(amti);
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
