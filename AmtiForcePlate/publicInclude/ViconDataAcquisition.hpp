#pragma once

#include "ForcePlateDataFrame.hpp"
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
    class ViconDataAcquisition;
}

class Acquisition::ViconDataAcquisition
{
public:
    static ViconDataAcquisition create();
    ViconDataAcquisition(ViconDataAcquisition&&) = default;
    ViconDataAcquisition(ViconDataAcquisition&) = delete;
    ~ViconDataAcquisition();

    ForcePlateDataFrame grabForcePlataDataFrame(const std::string &amti);
    static const std::string amti1;
    static const std::string amti2;

    void waitForFrame();
    std::vector<MarkerGlobalTranslationData> grabMarkerGlobalTranslation();

    const long subsampleCount;

    const std::vector<std::string> markerNames;
    const std::string subjectName;

private:
    std::unique_ptr<ViconDataStreamClient::DataStreamClientFacade> client;

    ViconDataAcquisition(std::unique_ptr<ViconDataStreamClient::DataStreamClientFacade>&& client, long subsampleCount, std::vector<std::string>&& markerNames, std::string&& subjectName);
    static void waitForFrame(ViconDataStreamClient::DataStreamClientFacade& client);
};