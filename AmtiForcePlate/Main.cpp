#include "DataStreamClient.h"

#include <iostream>
#include "ForcePlateDataAcquisition.hpp"

using namespace Acquisition;

int main()
{
    try
    {
        ForcePlateDataAcquisition forcePlateDataAcquisition;

        const std::string &amti = ForcePlateDataAcquisition::amti1;

        while (true)
        {
            std::vector<ForcePlateData> data = forcePlateDataAcquisition.grabDirect(amti);
            for (const ForcePlateData &dataPoint : data)
            {
                std::cout << dataPoint.frameNumber << ", " << dataPoint.subsampleNumber << ", " << amti << ", "
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
