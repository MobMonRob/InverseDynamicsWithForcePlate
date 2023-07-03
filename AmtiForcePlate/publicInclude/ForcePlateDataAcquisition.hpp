#pragma once

#include "ForcePlateDataFrame.hpp"

#include <string>
#include <memory>

namespace ViconDataStreamClient
{
    class DataStreamClientFacade;
}

namespace Acquisition
{
    class ForcePlateDataAcquisition;
}

class Acquisition::ForcePlateDataAcquisition
{
public:
    ForcePlateDataAcquisition();
    ~ForcePlateDataAcquisition();

    ForcePlateDataFrame grabDirect(const std::string &amti);
    static const std::string amti1;
    static const std::string amti2;

private:
    std::unique_ptr<ViconDataStreamClient::DataStreamClientFacade> client;
    const long subsampleCount;

    static ViconDataStreamClient::DataStreamClientFacade &setupClient(ViconDataStreamClient::DataStreamClientFacade &client);
    static void waitForFrame(ViconDataStreamClient::DataStreamClientFacade &client);
};
