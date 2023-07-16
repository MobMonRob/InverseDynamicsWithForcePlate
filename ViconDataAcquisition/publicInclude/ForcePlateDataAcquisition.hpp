#pragma once

#include "ForcePlateData.hpp"

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
    static ForcePlateDataAcquisition create(const std::string& hostname, const std::string &amti);
    ForcePlateDataAcquisition(ForcePlateDataAcquisition &&) = default;
    ForcePlateDataAcquisition(const ForcePlateDataAcquisition &) = delete;
    ~ForcePlateDataAcquisition();

    uint getFrame();
    const std::vector<ForcePlateData>& getForcePlateDataVectorCache();
    void updateForcePlateDataVectorCache();

    const std::string amti;

    static const std::string defaultHostname;
    static const std::string defaultAMTI;

private:
    std::unique_ptr<ViconDataStreamClient::DataStreamClientFacade> client;
    std::vector<ForcePlateData> forcePlateDataVectorCache;
    const uint subsampleCount;

    ForcePlateDataAcquisition(std::unique_ptr<ViconDataStreamClient::DataStreamClientFacade> &&client, std::vector<ForcePlateData>&& forcePlateDataVectorCache, const std::string &amti);
};
