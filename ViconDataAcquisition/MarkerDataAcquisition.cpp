#include "MarkerDataAcquisition.hpp"

#include "../DataStreamClientFacade.hpp"

#include <iostream>
#include <thread>
#include <chrono>

using namespace Acquisition;
using namespace ViconDataStreamClient;

const std::string MarkerDataAcquisition::defaultHostname("192.168.10.1:801");
const std::string MarkerDataAcquisition::defaultSubjectName("Tip");

MarkerDataAcquisition::MarkerDataAcquisition(std::unique_ptr<ViconDataStreamClient::DataStreamClientFacade> &&client, std::vector<std::string> &&markerNames, const std::string &subjectName, std::vector<MarkerGlobalTranslationData>&& markerGlobalTranslationVectorCache)
    : client(std::move(client)),
      markerNames(std::move(markerNames)),
      subjectName(subjectName),
      markerGlobalTranslationVectorCache(std::move(markerGlobalTranslationVectorCache)),
      markerNamesCount(markerNames.size())
{
}

MarkerDataAcquisition::~MarkerDataAcquisition()
{
}

uint MarkerDataAcquisition::getFrame()
{
    client->getFrame();
    return client->getFrameNumber();
}

const std::vector<MarkerGlobalTranslationData>& MarkerDataAcquisition::getMarkerGlobalTranslationVectorCache()
{
    return markerGlobalTranslationVectorCache;
}

void MarkerDataAcquisition::updateMarkerGlobalTranslationVectorCache()
{
    for (uint i = 0; i < markerNamesCount; ++i)
    {
        const std::string &markerName(markerNames[i]);
        MarkerGlobalTranslationData& data(markerGlobalTranslationVectorCache[i]);

        data.occluded = client->getLastWasOccluded();

        std::array<double, 3> translation = client->getMarkerGlobalTranslation(subjectName, markerName);
        data.x = translation[0];
        data.y = translation[1];
        data.z = translation[2];
    }
}

MarkerDataAcquisition MarkerDataAcquisition::create(const std::string& hostname, const std::string& subjectName)
{
    std::cout << "subjectName: " << subjectName << std::endl;

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

    client->enableMarkerData();

    client->clearSubjectFilter();
    client->addToSubjectFilter(subjectName);

    client->getFrame();
    client->getFrame();

    const uint markerCount = client->getMarkerCount(subjectName);
    std::cout << "markerCount: " << markerCount << std::endl;

    std::vector<std::string> markerNames;
    markerNames.reserve(markerCount);
    for (uint i = 0; i < markerCount; ++i)
    {
        std::string markerName = client->getMarkerName(subjectName, i);
        markerNames.push_back(markerName);

        std::cout << markerName << std::endl;
    }

    std::vector<MarkerGlobalTranslationData> markerGlobalTranslationVectorCache(markerCount);

    //////////////////////////////////////////////////

    MarkerDataAcquisition markerDataAcquisition(std::move(client), std::move(markerNames), subjectName, std::move(markerGlobalTranslationVectorCache));

    return markerDataAcquisition;
}
