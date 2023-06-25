#pragma once
#include <string>

#include "DataStreamClient.h"

namespace ViconDataStreamClient
{
    class DataStreamClientFacade;
}

class ViconDataStreamClient::DataStreamClientFacade
{
public:
    void enableDeviceData();
    void setStreamMode(ViconDataStreamSDK::CPP::StreamMode::Enum mode);
    bool getFrame();
    long getDeviceOutputSubsamples(const std::string &deviceName, const std::string &deviceOutputName);
    long getFrameNumber();
    void connect(const std::string &hostname, long timeoutInMs);
    bool isConnected();
    void setBufferSize(unsigned int bufferSize);
    double getDeviceOutputValue(const std::string &deviceName, const std::string &deviceOutputComponentName);
    double getDeviceOutputValue(const std::string &deviceName, const std::string &deviceOutputComponentName, const unsigned int subsample);

private:
    ViconDataStreamSDK::CPP::Client innerDataStreamClient;
    std::string hostname;
};