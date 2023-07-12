#include "DataStreamClientFacade.hpp"

#include <iostream>
#include <thread>
#include <chrono>

using namespace ViconDataStreamSDK::CPP;
using namespace ViconDataStreamClient;

void DataStreamClientFacade::enableDeviceData()
{
    Output_EnableDeviceData result = innerDataStreamClient.EnableDeviceData();
    // std::cout << "Enable marker data: " << result.getResult().toString() << std::endl;
    if (result.Result == Result::Enum::NotConnected)
    {
        throw std::runtime_error("The client is not connected!");
    }
    else if (result.Result != Result::Enum::Success)
    {
        throw std::runtime_error("Enable device data failed due to unknown reason!");
    }
}

void DataStreamClientFacade::setStreamMode(StreamMode::Enum mode)
{
    Output_SetStreamMode result = innerDataStreamClient.SetStreamMode(mode);
    if (result.Result == Result::Enum::NotConnected)
    {
        throw std::runtime_error("setStreamMode() only allowed if the client is connected!");
    }
}

bool DataStreamClientFacade::getFrame()
{
    Output_GetFrame res = innerDataStreamClient.GetFrame();
    Result::Enum result = res.Result;
    if (result == Result::Enum::NotConnected)
    {
        throw std::runtime_error("getFrame() but client is not connected!");
    }
    return result == Result::Enum::Success;
}

long DataStreamClientFacade::getDeviceOutputSubsamples(const std::string &deviceName, const std::string &deviceOutputName)
{
    Output_GetDeviceOutputSubsamples result = innerDataStreamClient.GetDeviceOutputSubsamples(deviceName, deviceOutputName);
    if (result.Result == Result::Enum::InvalidIndex)
    {
        throw std::invalid_argument("getDeviceOutputName() but deviceName is invalid!");
    }
    else if (result.Result == Result::Enum::NoFrame)
    {
        throw std::runtime_error("getDeviceOutputSubsamples() but no frame available!");
    }
    else if (result.Result == Result::Enum::NotConnected)
    {
        throw std::runtime_error("getDeviceOutputSubsamples but client is not connected!");
    }

    if (result.Occluded)
    {
        return -1;
    }

    return result.DeviceOutputSubsamples;
}

long DataStreamClientFacade::getFrameNumber()
{
    Output_GetFrameNumber frameNumber = innerDataStreamClient.GetFrameNumber();
    if (frameNumber.Result == Result::Enum::NotConnected)
    {
        throw std::runtime_error("getFrameNumber() but client not connected!");
    }
    return frameNumber.FrameNumber;
}

bool DataStreamClientFacade::isConnected()
{
    return innerDataStreamClient.IsConnected().Connected;
}

void DataStreamClientFacade::connect(const std::string &hostname, long timeoutInMs)
{
    this->hostname = hostname;
    if (hostname.empty())
    {
        throw std::invalid_argument("connect() with argument==null not allowed!");
    }
    auto timeout = std::chrono::system_clock::now() + std::chrono::milliseconds(timeoutInMs);
    int trials = 0;
    // std::cout << "connect() ..." << std::endl;
    while (!isConnected() && std::chrono::system_clock::now() < timeout)
    {
        Output_Connect result = innerDataStreamClient.Connect(hostname);

        if (result.Result == Result::Enum::InvalidHostName)
        {
            std::cout << "Vicon DataStream client connection failed (" << trials++ << "): Invalid hostname \"" << hostname << "\"!" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        else if (result.Result == Result::Enum::Success)
        {
            std::cout << "Vicon DataStream client connected!" << std::endl;
            getFrame();
        }
        else if (result.Result == Result::Enum::ClientConnectionFailed)
        {
            std::cout << "Vicon client connection failed (" << trials++ << ")!" << std::endl;
        }
    }
}

void DataStreamClientFacade::setBufferSize(unsigned int bufferSize)
{
    innerDataStreamClient.SetBufferSize(bufferSize);
}

double DataStreamClientFacade::getDeviceOutputValue(const std::string &deviceName, const std::string &deviceOutputComponentName)
{
    Output_GetDeviceOutputValue result = innerDataStreamClient.GetDeviceOutputValue(deviceName, deviceOutputComponentName);
    if (result.Result == Result::Enum::InvalidIndex)
    {
        throw std::invalid_argument("getDeviceOutputName() but deviceName is invalid!");
    }
    else if (result.Result == Result::Enum::NoFrame)
    {
        throw std::runtime_error("getDeviceOutputName() but no frame available!");
    }
    else if (result.Result == Result::Enum::NotConnected)
    {
        throw std::runtime_error("getDeviceOutputName but client is not connected!!");
    }
    double deviceOutputValue = result.Value;
    if (result.Occluded)
    {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return deviceOutputValue;
}

double DataStreamClientFacade::getDeviceOutputValue(const std::string &deviceName, const std::string &deviceOutputComponentName, const unsigned int subsample)
{
    Output_GetDeviceOutputValue result = innerDataStreamClient.GetDeviceOutputValue(deviceName, deviceOutputComponentName, subsample);
    if (result.Result == Result::Enum::InvalidIndex)
    {
        throw std::invalid_argument("getDeviceOutputName() but deviceName is invalid!");
    }
    else if (result.Result == Result::Enum::NoFrame)
    {
        throw std::runtime_error("getDeviceOutputName() but no frame available!");
    }
    else if (result.Result == Result::Enum::NotConnected)
    {
        throw std::runtime_error("getDeviceOutputName() but client is not connected!");
    }
    else if (result.Result != Result::Enum::Success)
    {
        throw std::runtime_error("getDeviceOutputName() but no success!");
    }
    double deviceOutputValue = result.Value;
    if (result.Occluded)
    {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return deviceOutputValue;
}

void DataStreamClientFacade::enableMarkerData()
{
    Output_EnableMarkerData output = innerDataStreamClient.EnableMarkerData();
    Result::Enum result = output.Result;
    if (result == Result::Enum::NotConnected)
    {
        throw std::runtime_error("The client is not connected!");
    }
    else if (result != Result::Enum::Success)
    {
        throw std::runtime_error("enableMarkerData() failed due to unknown reason!");
    }
}

