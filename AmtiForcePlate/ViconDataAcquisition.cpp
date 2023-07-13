#include "ViconDataAcquisition.hpp"

#include "../DataStreamClientFacade.hpp"

#include <iostream>
#include <thread>
#include <chrono>

using namespace Acquisition;
using namespace ViconDataStreamClient;

const std::string ViconDataAcquisition::amti1("AMTI 1");
const std::string ViconDataAcquisition::amti2("AMTI 2");

ViconDataAcquisition::ViconDataAcquisition(std::unique_ptr<ViconDataStreamClient::DataStreamClientFacade>&& client, uint subsampleCount, std::vector<std::string>&& markerNames, std::string&& subjectName)
    : client(std::move(client)),
      subsampleCount(subsampleCount),
      markerNames(std::move(markerNames)),
      subjectName(std::move(subjectName))
{
}

ViconDataAcquisition::~ViconDataAcquisition()
{
}

uint ViconDataAcquisition::waitForFrame()
{
    waitForFrame(*client);
    return client->getFrameNumber();
}

void ViconDataAcquisition::waitForFrame(ViconDataStreamClient::DataStreamClientFacade &client)
{
    while (!client.getFrame())
    {
        std::cout << "waiting" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

std::vector<ForcePlateData> ViconDataAcquisition::grabForcePlataDataFrame(const std::string &amti)
{
    std::vector<ForcePlateData> forcePlateDataVector;
    forcePlateDataVector.reserve(subsampleCount);

    for (uint subsample = 0; subsample < subsampleCount; ++subsample)
    {
        double fx = client->getDeviceOutputValue(amti, "Fx", subsample); // N
        double fy = client->getDeviceOutputValue(amti, "Fy", subsample);
        double fz = client->getDeviceOutputValue(amti, "Fz", subsample);
        double mx = client->getDeviceOutputValue(amti, "MX", subsample); // Nm bzw. J
        double my = client->getDeviceOutputValue(amti, "MY", subsample);
        double mz = client->getDeviceOutputValue(amti, "MZ", subsample);

        ForcePlateData data(fx, fy, fz, mx, my, mz);

        forcePlateDataVector.push_back(std::move(data));
    }

    return forcePlateDataVector;
}

using namespace ViconDataStreamSDK::CPP;
using namespace ViconDataStreamClient;

ViconDataAcquisition ViconDataAcquisition::create()
{
    std::unique_ptr<ViconDataStreamClient::DataStreamClientFacade> client(std::make_unique<ViconDataStreamClient::DataStreamClientFacade>());

    std::string hostname = "192.168.10.1:801";

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
    // client->enableDebugData();

    waitForFrame(*client);
    waitForFrame(*client);

    uint subsampleCount = client->getDeviceOutputSubsamples(ViconDataAcquisition::amti2, "Fz");

    //////////////////////////////////////////////////

    client->enableMarkerData();

    std::string subjectName = "Tip";

    client->clearSubjectFilter();
    client->addToSubjectFilter(subjectName);

    waitForFrame(*client);
    waitForFrame(*client);

    uint markerCount = client->getMarkerCount(subjectName);
    std::cout << "markerCount: " << markerCount << std::endl;

    std::vector<std::string> markerNames;
    markerNames.reserve(markerCount);

    for (uint i = 0; i < markerCount; ++i)
    {
        std::string markerName = client->getMarkerName(subjectName, i);
        markerNames.push_back(markerName);

        std::cout << markerName << std::endl;
    }

    //////////////////////////////////////////////////

    ViconDataAcquisition viconDataAcquisition(std::move(client), subsampleCount, std::move(markerNames), std::move(subjectName));

    return viconDataAcquisition;
}

std::vector<MarkerGlobalTranslationData> ViconDataAcquisition::grabMarkerGlobalTranslation()
{
    std::vector<MarkerGlobalTranslationData> dataVector;
    dataVector.reserve(markerNames.size());

    for (const std::string& markerName : markerNames)
    {
        std::array<double, 3> translation = client->getMarkerGlobalTranslation(subjectName, markerName);
        bool occluded = client->getLastWasOccluded();
        double x = translation[0];
        double y = translation[1];
        double z = translation[2];
        MarkerGlobalTranslationData data(occluded, x, y, z);
        dataVector.push_back(std::move(data));
    }

    return dataVector;
}
