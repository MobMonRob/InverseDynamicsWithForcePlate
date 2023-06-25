#pragma once

#include "DataStreamClientFacade.hpp"

namespace Acquisition
{
    class ForcePlateDataAcquisition;
}

class Acquisition::ForcePlateDataAcquisition
{
public:
    ForcePlateDataAcquisition();

    void grabDirect();

private:
    ViconDataStreamClient::DataStreamClientFacade client;
    const std::vector<std::string> amtis;
    const long subsampleCount;

    static ViconDataStreamClient::DataStreamClientFacade &setupClient(ViconDataStreamClient::DataStreamClientFacade &client);
    static void waitForFrame(ViconDataStreamClient::DataStreamClientFacade &client);
};