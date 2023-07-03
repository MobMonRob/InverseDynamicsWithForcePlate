#pragma once

#include "DataStreamClientFacade.hpp"
#include "ForcePlateData.hpp"

#include <string>

namespace Acquisition
{
    class ForcePlateDataAcquisition;
}

class Acquisition::ForcePlateDataAcquisition
{
public:
    ForcePlateDataAcquisition();

    std::vector<ForcePlateData> grabDirect(const std::string &amti);
    static const std::string amti1;
    static const std::string amti2;

private:
    ViconDataStreamClient::DataStreamClientFacade client;
    const long subsampleCount;

    static ViconDataStreamClient::DataStreamClientFacade &setupClient(ViconDataStreamClient::DataStreamClientFacade &client);
    static void waitForFrame(ViconDataStreamClient::DataStreamClientFacade &client);
};
