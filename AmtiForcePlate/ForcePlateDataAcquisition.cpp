#include "ForcePlateDataAcquisition.hpp"

#include <iostream>
#include <thread>
#include <chrono>

using namespace Acquisition;
using namespace ViconDataStreamClient;

ForcePlateDataAcquisition::ForcePlateDataAcquisition()
    : client(),
      amtis({"AMTI 1", "AMTI 2"}),
      subsampleCount(setupClient(client).getDeviceOutputSubsamples("AMTI 1", "Fx"))
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

// int prevNum = 0;

void ForcePlateDataAcquisition::grabDirect()
{
    waitForFrame(client);
    long frameNumber = client.getFrameNumber();

    // if (frameNumber - prevNum > 1) {
    //     std::cout << prevNum << "::" << frameNumber << std::endl;
    // }
    // prevNum = frameNumber;

    for (unsigned int subsample = 0; subsample < subsampleCount; ++subsample)
    {
        for (const auto &amti : amtis)
        {
            double fx = client.getDeviceOutputValue(amti, "Fx", subsample); // N
            double fy = client.getDeviceOutputValue(amti, "Fy", subsample);
            double fz = client.getDeviceOutputValue(amti, "Fz", subsample);
            double mx = client.getDeviceOutputValue(amti, "MX", subsample); // Nm bzw. J
            double my = client.getDeviceOutputValue(amti, "MY", subsample);
            double mz = client.getDeviceOutputValue(amti, "MZ", subsample);
            std::cout << frameNumber << ", " << subsample << ", " << amti << ", "
                      << fx << ", " << fy << ", " << fz << ", "
                      << mx << ", " << my << ", " << mz << std::endl;
        }
    }
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