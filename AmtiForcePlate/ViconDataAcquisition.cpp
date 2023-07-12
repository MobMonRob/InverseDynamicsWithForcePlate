#include "ViconDataAcquisition.hpp"

#include "../DataStreamClientFacade.hpp"

#include <iostream>
#include <thread>
#include <chrono>

using namespace Acquisition;
using namespace ViconDataStreamClient;

const std::string ViconDataAcquisition::amti1("AMTI 1");
const std::string ViconDataAcquisition::amti2("AMTI 2");

ViconDataAcquisition::ViconDataAcquisition(std::unique_ptr<ViconDataStreamClient::DataStreamClientFacade>&& client, long subsampleCount, std::vector<std::string>&& markerNames, std::string&& subjectName)
    : client(std::move(client)),
      subsampleCount(subsampleCount),
      markerNames(std::move(markerNames)),
      subjectName(std::move(subjectName))
{
}

ViconDataAcquisition::~ViconDataAcquisition()
{
}

void ViconDataAcquisition::waitForFrame()
{
    waitForFrame(*client);
}

void ViconDataAcquisition::waitForFrame(ViconDataStreamClient::DataStreamClientFacade &client)
{
    while (!client.getFrame())
    {
        std::cout << "waiting" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

ForcePlateDataFrame ViconDataAcquisition::grabForcePlataDataFrame(const std::string &amti)
{
    long frameNumber = client->getFrameNumber();

    std::vector<ForcePlateData> forcePlateDataVector(subsampleCount);

    for (unsigned int subsample = 0; subsample < subsampleCount; ++subsample)
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

    ForcePlateDataFrame forcePlateDataFrame(frameNumber, std::move(forcePlateDataVector));

    return forcePlateDataFrame;
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

    long subsampleCount = client->getDeviceOutputSubsamples(ViconDataAcquisition::amti2, "Fz");

    //////////////////////////////////////////////////

    client->enableMarkerData();
    ViconDataStreamSDK::CPP::Client& vicon(client->getInner());

    std::string subjectName = "Tip";

    vicon.ClearSubjectFilter();
    vicon.AddToSubjectFilter(subjectName);

    waitForFrame(*client);
    waitForFrame(*client);

    Output_GetMarkerCount output_GetMarkerCount = vicon.GetMarkerCount(subjectName);
    uint markerCount = output_GetMarkerCount.MarkerCount;
    std::cout << "markerCount: " << markerCount << std::endl;

    std::vector<std::string> markerNames;

    for (uint i = 0; i < markerCount; ++i)
    {
        Output_GetMarkerName output_GetMarkerName = vicon.GetMarkerName(subjectName, i);
        std::string markerName = output_GetMarkerName.MarkerName;
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

    for (const std::string& markerName : markerNames)
    {
        Output_GetMarkerGlobalTranslation output_GetMarkerGlobalTranslation = client->getInner().GetMarkerGlobalTranslation(subjectName, markerName);
        double x = output_GetMarkerGlobalTranslation.Translation[0];
        double y = output_GetMarkerGlobalTranslation.Translation[1];
        double z = output_GetMarkerGlobalTranslation.Translation[2];
        bool occluded = output_GetMarkerGlobalTranslation.Occluded;
        MarkerGlobalTranslationData data(x, y, z, occluded, markerName);
        dataVector.push_back(std::move(data));
    }

    return dataVector;
}
