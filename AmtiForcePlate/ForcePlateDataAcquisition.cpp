#include "ForcePlateDataAcquisition.hpp"

#include "../DataStreamClientFacade.hpp"

#include <iostream>
#include <thread>
#include <chrono>

using namespace Acquisition;
using namespace ViconDataStreamClient;

const std::string ForcePlateDataAcquisition::amti1("AMTI 1");
const std::string ForcePlateDataAcquisition::amti2("AMTI 2");

ForcePlateDataAcquisition::ForcePlateDataAcquisition(std::unique_ptr<ViconDataStreamClient::DataStreamClientFacade> &&client, uint subsampleCount)
    : client(std::move(client)),
      subsampleCount(subsampleCount)
{
}

ForcePlateDataAcquisition::~ForcePlateDataAcquisition()
{
}

uint ForcePlateDataAcquisition::waitForFrame()
{
    waitForFrame(*client);
    return client->getFrameNumber();
}

void ForcePlateDataAcquisition::waitForFrame(ViconDataStreamClient::DataStreamClientFacade &client)
{
    while (!client.getFrame())
    {
        std::cout << "waiting" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

std::vector<ForcePlateData> ForcePlateDataAcquisition::grabForcePlataDataFrame(const std::string &amti)
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

ForcePlateDataAcquisition ForcePlateDataAcquisition::create()
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

    uint subsampleCount = client->getDeviceOutputSubsamples(ForcePlateDataAcquisition::amti2, "Fz");

    //////////////////////////////////////////////////

    ForcePlateDataAcquisition ForcePlateDataAcquisition(std::move(client), subsampleCount);

    return ForcePlateDataAcquisition;
}
