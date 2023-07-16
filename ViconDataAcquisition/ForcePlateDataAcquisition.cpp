#include "ForcePlateDataAcquisition.hpp"

#include "../DataStreamClientFacade.hpp"

#include <iostream>

using namespace Acquisition;
using namespace ViconDataStreamClient;

const std::string ForcePlateDataAcquisition::defaultHostname("192.168.10.1:801");
const std::string ForcePlateDataAcquisition::defaultAMTI("AMTI 2");

ForcePlateDataAcquisition::ForcePlateDataAcquisition(std::unique_ptr<ViconDataStreamClient::DataStreamClientFacade> &&client, std::vector<ForcePlateData>&& forcePlateDataVectorCache, const std::string &amti)
    : client(std::move(client)),
      forcePlateDataVectorCache(std::move(forcePlateDataVectorCache)),
      amti(amti),
      subsampleCount(forcePlateDataVectorCache.size())
{
}

ForcePlateDataAcquisition::~ForcePlateDataAcquisition()
{
}

uint ForcePlateDataAcquisition::getFrame()
{
    client->getFrame();
    return client->getFrameNumber();
}

const std::vector<ForcePlateData>& ForcePlateDataAcquisition::getForcePlateDataVectorCache()
{
    return forcePlateDataVectorCache;
}

void ForcePlateDataAcquisition::updateForcePlateDataVectorCache()
{
    for (uint subsample = 0; subsample < subsampleCount; ++subsample)
    {
        ForcePlateData& data(forcePlateDataVectorCache[subsample]);
        data.fX = client->getDeviceOutputValue(amti, "Fx", subsample); // N
        data.fY = client->getDeviceOutputValue(amti, "Fy", subsample);
        data.fZ = client->getDeviceOutputValue(amti, "Fz", subsample);
        data.mX = client->getDeviceOutputValue(amti, "MX", subsample); // Nm bzw. J
        data.mY = client->getDeviceOutputValue(amti, "MY", subsample);
        data.mZ = client->getDeviceOutputValue(amti, "MZ", subsample);
    }
}

ForcePlateDataAcquisition ForcePlateDataAcquisition::create(const std::string& hostname, const std::string &amti)
{
    std::cout << "amti: " << amti << std::endl;

    std::unique_ptr<ViconDataStreamClient::DataStreamClientFacade> client(std::make_unique<ViconDataStreamClient::DataStreamClientFacade>());

    std::cout << "Try to connect to: " << hostname << std::endl;
    client->connect(hostname, 4000);
    if (client->isConnected())
    {
        std::cout << "is connected" << std::endl;
    }

    client->setBufferSize(1000);

    client->setStreamMode(ViconDataStreamSDK::CPP::StreamMode::Enum::ServerPush);

    //////////////////////////////////////////////////

    client->enableDeviceData();

    client->getFrame();
    client->getFrame();

    const uint subsampleCount = client->getDeviceOutputSubsamples(amti, "Fz");

    std::vector<ForcePlateData> forcePlateDataVectorCache(subsampleCount);

    //////////////////////////////////////////////////

    ForcePlateDataAcquisition ForcePlateDataAcquisition(std::move(client), std::move(forcePlateDataVectorCache), amti);

    return ForcePlateDataAcquisition;
}
