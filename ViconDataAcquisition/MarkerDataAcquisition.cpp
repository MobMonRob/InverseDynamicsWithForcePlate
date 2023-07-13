#include "MarkerDataAcquisition.hpp"

#include "../DataStreamClientFacade.hpp"

#include <iostream>
#include <thread>
#include <chrono>

using namespace Acquisition;
using namespace ViconDataStreamClient;

MarkerDataAcquisition::MarkerDataAcquisition(std::unique_ptr<ViconDataStreamClient::DataStreamClientFacade> &&client, std::vector<std::string> &&markerNames, std::string &&subjectName)
    : client(std::move(client)),
      markerNames(std::move(markerNames)),
      subjectName(std::move(subjectName))
{
}

MarkerDataAcquisition::~MarkerDataAcquisition()
{
}

uint MarkerDataAcquisition::waitForFrame()
{
    waitForFrame(*client);
    return client->getFrameNumber();
}

void MarkerDataAcquisition::waitForFrame(ViconDataStreamClient::DataStreamClientFacade &client)
{
    while (!client.getFrame())
    {
        std::cout << "waiting" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}


using namespace ViconDataStreamSDK::CPP;
using namespace ViconDataStreamClient;

MarkerDataAcquisition MarkerDataAcquisition::create()
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

    MarkerDataAcquisition markerDataAcquisition(std::move(client), std::move(markerNames), std::move(subjectName));

    return markerDataAcquisition;
}

std::vector<MarkerGlobalTranslationData> MarkerDataAcquisition::grabMarkerGlobalTranslation()
{
    std::vector<MarkerGlobalTranslationData> dataVector;
    dataVector.reserve(markerNames.size());

    for (const std::string &markerName : markerNames)
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
