#pragma once

#include <string>
#include <array>
#include <stdexcept>
#include <optional>

#include "DataStreamClient.h"

namespace ViconDataStreamClient
{
    class DataStreamClientFacade;
    class ResultException;
}

class ViconDataStreamClient::ResultException : public std::runtime_error {
public:
    ResultException(ViconDataStreamSDK::CPP::Result::Enum cause);
    const ViconDataStreamSDK::CPP::Result::Enum cause;
    static std::string causeToString(ViconDataStreamSDK::CPP::Result::Enum cause);
};

class ViconDataStreamClient::DataStreamClientFacade
{
public:
    ViconDataStreamSDK::CPP::Client& getInner() {return innerDataStreamClient;}

    bool getLastWasOccluded() {return lastWasOccluded;}

    void enableDeviceData();
    void setStreamMode(ViconDataStreamSDK::CPP::StreamMode::Enum mode);
    bool getFrame();
    uint getDeviceOutputSubsamples(const std::string &deviceName, const std::string &deviceOutputName);
    uint getFrameNumber();
    void connect(const std::string &hostname, const long timeoutInMs);
    bool isConnected();
    void setBufferSize(const uint bufferSize);
    double getDeviceOutputValue(const std::string &deviceName, const std::string &deviceOutputComponentName);
    double getDeviceOutputValue(const std::string &deviceName, const std::string &deviceOutputComponentName, const uint subsample);
    void enableMarkerData();
    uint getMarkerCount( const std::string  & subjectName );
    std::string getMarkerName( const std::string & subjectName, const uint  markerIndex );
    void clearSubjectFilter();
    void addToSubjectFilter( const std::string& subjectName );
    std::array<double, 3> getMarkerGlobalTranslation( const std::string & subjectName, const std::string & markerName );

private:
    bool lastWasOccluded;
    ViconDataStreamSDK::CPP::Client innerDataStreamClient;
    std::string hostname;

    static void ensureSuccess(ViconDataStreamSDK::CPP::Result::Enum result);
};
