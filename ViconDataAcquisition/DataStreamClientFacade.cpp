#include "DataStreamClientFacade.hpp"

#include <iostream>
#include <thread>
#include <chrono>

using namespace ViconDataStreamSDK::CPP;
using namespace ViconDataStreamClient;

ResultException::ResultException(ViconDataStreamSDK::CPP::Result::Enum cause)
    : std::runtime_error(causeToString(cause)), cause(cause)
{
}

std::string ResultException::causeToString(ViconDataStreamSDK::CPP::Result::Enum cause)
{
    switch (cause)
    {
    case Result::Enum::Unknown:
        return "Unknown";
    case Result::Enum::NotImplemented:
        return "NotImplemented";
    case Result::Enum::InvalidHostName:
        return "InvalidHostName";
    case Result::Enum::InvalidMulticastIP:
        return "InvalidMulticastIP";
    case Result::Enum::ClientAlreadyConnected:
        return "ClientAlreadyConnected";
    case Result::Enum::ClientConnectionFailed:
        return "ClientConnectionFailed";
    case Result::Enum::ServerAlreadyTransmittingMulticast:
        return "ServerAlreadyTransmittingMulticast";
    case Result::Enum::ServerNotTransmittingMulticast:
        return "ServerNotTransmittingMulticast";
    case Result::Enum::NoFrame:
        return "NoFrame";
    case Result::Enum::InvalidIndex:
        return "InvalidIndex";
    case Result::Enum::InvalidCameraName:
        return "InvalidCameraName";
    case Result::Enum::InvalidSubjectName:
        return "InvalidSubjectName";
    case Result::Enum::InvalidSegmentName:
        return "InvalidSegmentName";
    case Result::Enum::InvalidMarkerName:
        return "InvalidMarkerName";
    case Result::Enum::InvalidDeviceName:
        return "InvalidDeviceName";
    case Result::Enum::InvalidDeviceOutputName:
        return "InvalidDeviceOutputName";
    case Result::Enum::InvalidLatencySampleName:
        return "InvalidLatencySampleName";
    case Result::Enum::CoLinearAxes:
        return "CoLinearAxes";
    case Result::Enum::LeftHandedAxes:
        return "LeftHandedAxes";
    case Result::Enum::HapticAlreadySet:
        return "HapticAlreadySet";
    case Result::Enum::EarlyDataRequested:
        return "EarlyDataRequested";
    case Result::Enum::LateDataRequested:
        return "LateDataRequested";
    case Result::Enum::InvalidOperation:
        return "InvalidOperation";
    case Result::Enum::NotSupported:
        return "NotSupported";
    case Result::Enum::ConfigurationFailed:
        return "ConfigurationFailed";
    case Result::Enum::NotPresent:
        return "NotPresent";
    case Result::Enum::ArgumentOutOfRange:
        return "ArgumentOutOfRange";
    default:
        return "default: unknown";
    }
}

void DataStreamClientFacade::ensureSuccess(ViconDataStreamSDK::CPP::Result::Enum resultEnum)
{
    if (resultEnum != Result::Enum::Success)
    {
        throw ResultException(resultEnum);
    }
}

void DataStreamClientFacade::enableDeviceData()
{
    Output_EnableDeviceData output = innerDataStreamClient.EnableDeviceData();
    ensureSuccess(output.Result);
}

void DataStreamClientFacade::setStreamMode(StreamMode::Enum mode)
{
    Output_SetStreamMode output = innerDataStreamClient.SetStreamMode(mode);
    ensureSuccess(output.Result);
}

bool DataStreamClientFacade::getFrame()
{
    Output_GetFrame output = innerDataStreamClient.GetFrame();
    ensureSuccess(output.Result);
    return true;
}

uint DataStreamClientFacade::getDeviceOutputSubsamples(const std::string &deviceName, const std::string &deviceOutputName)
{
    Output_GetDeviceOutputSubsamples output = innerDataStreamClient.GetDeviceOutputSubsamples(deviceName, deviceOutputName);
    ensureSuccess(output.Result);

    lastWasOccluded = output.Occluded;

    return output.DeviceOutputSubsamples;
}

uint DataStreamClientFacade::getFrameNumber()
{
    Output_GetFrameNumber output = innerDataStreamClient.GetFrameNumber();
    ensureSuccess(output.Result);

    return output.FrameNumber;
}

bool DataStreamClientFacade::isConnected()
{
    return innerDataStreamClient.IsConnected().Connected;
}

void DataStreamClientFacade::connect(const std::string &hostname, const long timeoutInMs)
{
    this->hostname = hostname;
    if (hostname.empty())
    {
        throw std::invalid_argument("connect() with argument==null not allowed!");
    }
    auto timeout = std::chrono::system_clock::now() + std::chrono::milliseconds(timeoutInMs);
    uint trials = 0;
    // std::cout << "connect() ..." << std::endl;
    while (!isConnected() && std::chrono::system_clock::now() < timeout)
    {
        Output_Connect output = innerDataStreamClient.Connect(hostname);

        if (output.Result == Result::Enum::InvalidHostName)
        {
            std::cout << "Vicon DataStream client connection failed (" << trials++ << "): Invalid hostname \"" << hostname << "\"!" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        else if (output.Result == Result::Enum::Success)
        {
            std::cout << "Vicon DataStream client connected!" << std::endl;
            getFrame();
        }
        else if (output.Result == Result::Enum::ClientConnectionFailed)
        {
            std::cout << "Vicon client connection failed (" << trials++ << ")!" << std::endl;
        }
    }
}

void DataStreamClientFacade::setBufferSize(const uint bufferSize)
{
    innerDataStreamClient.SetBufferSize(bufferSize);
}

double DataStreamClientFacade::getDeviceOutputValue(const std::string &deviceName, const std::string &deviceOutputComponentName)
{
    Output_GetDeviceOutputValue output = innerDataStreamClient.GetDeviceOutputValue(deviceName, deviceOutputComponentName);
    ensureSuccess(output.Result);

    lastWasOccluded = output.Occluded;

    return output.Value;
}

double DataStreamClientFacade::getDeviceOutputValue(const std::string &deviceName, const std::string &deviceOutputComponentName, const unsigned int subsample)
{
    Output_GetDeviceOutputValue output = innerDataStreamClient.GetDeviceOutputValue(deviceName, deviceOutputComponentName, subsample);
    ensureSuccess(output.Result);

    lastWasOccluded = output.Occluded;

    return output.Value;
}

void DataStreamClientFacade::enableMarkerData()
{
    Output_EnableMarkerData output = innerDataStreamClient.EnableMarkerData();
    ensureSuccess(output.Result);
}

uint DataStreamClientFacade::getMarkerCount(const std::string &subjectName)
{
    Output_GetMarkerCount output = innerDataStreamClient.GetMarkerCount(subjectName);
    ensureSuccess(output.Result);

    return output.MarkerCount;
}

std::string DataStreamClientFacade::getMarkerName(const std::string &subjectName, const uint markerIndex)
{
    Output_GetMarkerName output = innerDataStreamClient.GetMarkerName(subjectName, markerIndex);
    ensureSuccess(output.Result);

    return output.MarkerName;
}

void DataStreamClientFacade::clearSubjectFilter()
{
    Output_ClearSubjectFilter output = innerDataStreamClient.ClearSubjectFilter();
    ensureSuccess(output.Result);
}

void DataStreamClientFacade::addToSubjectFilter(const std::string &subjectName)
{
    Output_AddToSubjectFilter output = innerDataStreamClient.AddToSubjectFilter(subjectName);
    ensureSuccess(output.Result);
}

std::array<double, 3> DataStreamClientFacade::getMarkerGlobalTranslation(const std::string &subjectName, const std::string &markerName)
{
    Output_GetMarkerGlobalTranslation output = innerDataStreamClient.GetMarkerGlobalTranslation(subjectName, markerName);
    ensureSuccess(output.Result);

    lastWasOccluded = output.Occluded;

    return std::array<double, 3>({output.Translation[0], output.Translation[1], output.Translation[2]});
}
