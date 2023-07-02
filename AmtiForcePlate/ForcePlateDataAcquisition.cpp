#include "ForcePlateDataAcquisition.hpp"

#include <iostream>
#include <thread>
#include <chrono>
#include <vector>

using namespace Acquisition;
using namespace ViconDataStreamClient;

const std::string ForcePlateDataAcquisition::amti1("AMTI 1");
const std::string ForcePlateDataAcquisition::amti2("AMTI 2");

ForcePlateDataAcquisition::ForcePlateDataAcquisition()
    : client(),
      subsampleCount(setupClient(client).getDeviceOutputSubsamples(ForcePlateDataAcquisition::amti1, "Fx"))
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

std::vector<ForcePlateData> ForcePlateDataAcquisition::grabDirect(const std::string &amti)
{
    waitForFrame(client);
    long frameNumber = client.getFrameNumber();

    std::vector<ForcePlateData> forcePlateDataVector(subsampleCount);

    for (unsigned int subsample = 0; subsample < subsampleCount; ++subsample)
    {
        double fx = client.getDeviceOutputValue(amti, "Fx", subsample); // N
        double fy = client.getDeviceOutputValue(amti, "Fy", subsample);
        double fz = client.getDeviceOutputValue(amti, "Fz", subsample);
        double mx = client.getDeviceOutputValue(amti, "MX", subsample); // Nm bzw. J
        double my = client.getDeviceOutputValue(amti, "MY", subsample);
        double mz = client.getDeviceOutputValue(amti, "MZ", subsample);

        ForcePlateData data(ForcePlateData(frameNumber, subsample, fx, fy, fz, mx, my, mz));

        forcePlateDataVector.push_back(data);
    }
    return forcePlateDataVector;
}

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
    // client.enableDebugData();

    client.setBufferSize(100);

    // Tut manchmal.
    client.setStreamMode(ViconDataStreamSDK::CPP::StreamMode::Enum::ServerPush);
    //

    waitForFrame(client);
    waitForFrame(client);

    return client;
}