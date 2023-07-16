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
    static MarkerDataAcquisition create(const std::string& hostname, const std::string& subjectName);
    MarkerDataAcquisition(MarkerDataAcquisition &&) = default;
    MarkerDataAcquisition(const MarkerDataAcquisition &) = delete;
    ~MarkerDataAcquisition();

    uint getFrame();
    const std::vector<MarkerGlobalTranslationData>& getMarkerGlobalTranslationVectorCache();
    void updateMarkerGlobalTranslationVectorCache();

    const std::vector<std::string> markerNames;
    const std::string subjectName;

    static const std::string defaultHostname;
    static const std::string defaultSubjectName;

private:
    std::unique_ptr<ViconDataStreamClient::DataStreamClientFacade> client;
    std::vector<MarkerGlobalTranslationData> markerGlobalTranslationVectorCache;
    const uint markerNamesCount;

    MarkerDataAcquisition(std::unique_ptr<ViconDataStreamClient::DataStreamClientFacade> &&client, std::vector<std::string> &&markerNames, const std::string &subjectName, std::vector<MarkerGlobalTranslationData>&& markerGlobalTranslationVectorCache);
};
