#include "DataStreamClient.h"

#include <iostream>
#include "ForcePlateDataAcquisition.hpp"

using namespace Acquisition;

int main()
{
    // TODO: Add several filters, for example moving average filter.
    try
    {
        ForcePlateDataAcquisition forcePlateDataAcquisition;

        while (true)
        {
            forcePlateDataAcquisition.grabDirect();
        }
    }
    catch (const std::exception &e)
    {
        std::cout << e.what();
    }

    return 0;
}
