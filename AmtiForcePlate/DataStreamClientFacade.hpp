#pragma once
#include <string>
#include <array>

#include "DataStreamClient.h"

namespace ViconDataStreamClient
{
    class DataStreamClientFacade;
}

class ViconDataStreamClient::DataStreamClientFacade
{
public:
    ViconDataStreamSDK::CPP::Client getInner() {return innerDataStreamClient;}

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
    void enableMarkerData();

private:
    ViconDataStreamSDK::CPP::Client innerDataStreamClient;
    std::string hostname;
};
