#include "ForcePlateDataAcquisition.hpp"

#include "../DataStreamClientFacade.hpp"

#include <iostream>
#include <thread>
#include <chrono>
#include <vector>

using namespace Acquisition;
using namespace ViconDataStreamClient;

const std::string ForcePlateDataAcquisition::amti1("AMTI 1");
const std::string ForcePlateDataAcquisition::amti2("AMTI 2");

ForcePlateDataAcquisition::ForcePlateDataAcquisition()
    : client(std::make_unique<ViconDataStreamClient::DataStreamClientFacade>()),
      subsampleCount(setupClient(*client).getDeviceOutputSubsamples(ForcePlateDataAcquisition::amti2, "Fz"))
{
}

ForcePlateDataAcquisition::~ForcePlateDataAcquisition()
{
}

void ForcePlateDataAcquisition::waitForFrame(ViconDataStreamClient::DataStreamClientFacade &client)
{
    while (!client.getFrame())
    {
        std::cout << "waiting" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

ForcePlateDataFrame ForcePlateDataAcquisition::grabDirect(const std::string &amti)
{
    waitForFrame(*client);

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

ViconDataStreamClient::DataStreamClientFacade &ForcePlateDataAcquisition::setupClient(ViconDataStreamClient::DataStreamClientFacade &client)
{
    std::string hostname = "192.168.10.1:801";

    std::cout << "Try to connect to: " << hostname << std::endl;
    client.connect(hostname, 4000);
    if (client.isConnected())
    {
        std::cout << "is connected" << std::endl;
    }

    client.enableDeviceData();
    // client->enableDebugData();

    client.setBufferSize(100);

    client.setStreamMode(ViconDataStreamSDK::CPP::StreamMode::Enum::ServerPush);

    waitForFrame(client);
    waitForFrame(client);

    //////////////////////////////////////////////////

    client.enableMarkerData();
    ViconDataStreamSDK::CPP::Client vicon = client.getInner();

    std::string subjectName = "Tip";

    vicon.ClearSubjectFilter();
    vicon.AddToSubjectFilter(subjectName);

    Output_GetMarkerCount output_GetMarkerCount = vicon.GetMarkerCount(subjectName);
    uint markerCount = output_GetMarkerCount.MarkerCount;

    std::cout << "markerCount: " << markerCount << std::endl;

    for (;;)
    {
        waitForFrame(client);

        for (uint i = 0; i < markerCount; ++i)
        {
            Output_GetMarkerName output_GetMarkerName = vicon.GetMarkerName(subjectName, i);
            std::string markerName = output_GetMarkerName.MarkerName;

            Output_GetMarkerGlobalTranslation output_GetMarkerGlobalTranslation = vicon.GetMarkerGlobalTranslation(subjectName, markerName);
            double x = output_GetMarkerGlobalTranslation.Translation[0];
            // occluded auch

            std::cout << markerName << " " << x << std::endl;

            // Erst mal ausprobieren mit zwei Clients.
            // Sonst Facade mehrteilen.
            // Oder einfach alles zu einem Matschball formen.
        }
    }

    //////////////////////////////////////////////////

    return client;
}
