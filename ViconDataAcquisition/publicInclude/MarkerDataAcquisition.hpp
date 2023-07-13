#pragma once

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
    class MarkerDataAcquisition;
}

class Acquisition::MarkerDataAcquisition
{
public:
    static MarkerDataAcquisition create();
    MarkerDataAcquisition(MarkerDataAcquisition &&) = default;
    MarkerDataAcquisition(MarkerDataAcquisition &) = delete;
    ~MarkerDataAcquisition();

    uint waitForFrame();
    std::vector<MarkerGlobalTranslationData> grabMarkerGlobalTranslation();

    const std::vector<std::string> markerNames;
    const std::string subjectName;

private:
    std::unique_ptr<ViconDataStreamClient::DataStreamClientFacade> client;

    MarkerDataAcquisition(std::unique_ptr<ViconDataStreamClient::DataStreamClientFacade> &&client, std::vector<std::string> &&markerNames, std::string &&subjectName);
    static void waitForFrame(ViconDataStreamClient::DataStreamClientFacade &client);
};
