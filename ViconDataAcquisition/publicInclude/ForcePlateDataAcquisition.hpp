#pragma once

#include "ForcePlateData.hpp"
#include "MarkerGlobalTranslationData.hpp"

#include <string>
#include <memory>
#include <vector>

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
    static ForcePlateDataAcquisition create();
    ForcePlateDataAcquisition(ForcePlateDataAcquisition &&) = default;
    ForcePlateDataAcquisition(ForcePlateDataAcquisition &) = delete;
    ~ForcePlateDataAcquisition();

    std::vector<ForcePlateData> grabForcePlataDataFrame(const std::string &amti);
    static const std::string amti1;
    static const std::string amti2;

    uint waitForFrame();
    const uint subsampleCount;

private:
    std::unique_ptr<ViconDataStreamClient::DataStreamClientFacade> client;

    ForcePlateDataAcquisition(std::unique_ptr<ViconDataStreamClient::DataStreamClientFacade> &&client, uint subsampleCount);
    static void waitForFrame(ViconDataStreamClient::DataStreamClientFacade &client);
};
