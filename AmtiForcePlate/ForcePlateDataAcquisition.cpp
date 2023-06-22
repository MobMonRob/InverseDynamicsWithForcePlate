#include "DataStreamClient.h"
#include <iostream>
#include <thread>
#include <chrono>

using namespace ViconDataStreamSDK::CPP;

void enableDeviceData()
{
    Output_EnableDeviceData result = client.EnableDeviceData();
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

void setStreamMode(StreamMode::Enum mode)
{
    Output_SetStreamMode result = client.SetStreamMode(mode);
    if (result.Result == Result::Enum::NotConnected)
    {
        throw std::runtime_error("setStreamMode() only allowed if the client is connected!");
    }
}

bool getFrame()
{
    Output_GetFrame res = client.GetFrame();
    Result::Enum result = res.Result;
    if (result == Result::Enum::NotConnected)
    {
        throw std::runtime_error("getFrame() but client is not connected!");
    }
    return result == Result::Enum::Success;
}

void waitForFrame(Client &client)
{
    while (!client.getFrame())
    {
        std::cout << "waiting" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

long getDeviceOutputSubsamples(const std::string &deviceName, const std::string &deviceOutputName)
{
    Output_GetDeviceOutputSubsamples result = client.GetDeviceOutputSubsamples(deviceName, deviceOutputName);
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

long getFrameNumber()
{
    Output_GetFrameNumber frameNumber = client.GetFrameNumber();
    if (frameNumber.Result == Result::Enum::NotConnected)
    {
        throw std::runtime_error("getFrameNumber() but client not connected!");
    }
    return frameNumber.FrameNumber;
}

void grabDirect(Client &client)
{
    waitForFrame(client);
    std::vector<std::string> amtis = {"AMTI 1", "AMTI 2"};
    long subsampleCount = client.getDeviceOutputSubsamples("AMTI 1", "Fx");

    for (int frame = 0; frame < 100; ++frame)
    {
        waitForFrame(client);
        long frameNumber = client.getFrameNumber();
        for (int subsample = 0; subsample < subsampleCount; ++subsample)
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
}

DataStreamClient setupClient()
{
    DataStreamClient client;
    std::string hostname = "192.168.10.1:801";

    std::cout << "Try to connect to: " << hostname << std::endl;
    client.connect(hostname, 4000);
    if (client.isConnected())
    {
        std::cout << "is connected" << std::endl;
    }

    client.enableDeviceData();
    // client.enableDebugData();

    //
    client.setBufferSize(100);

    // Tut manchmal.
    client.setStreamMode(StreamMode_Enum::ServerPush);
    //

    client.getFrame();
    client.getFrame();

    return client;
}

int main()
{
    DataStreamClient client = setupClient();

    grabDirect(client);

    return 0;
}

// 	g++ -cpp "./ForcePlateDataAcquisition.cpp" -I"./ViconLib1.10" -Wl,--start-group -L"./ViconLib1.10" -l:libboost_atomic-mt.so.1.58.0 -l:libboost_chrono-mt.so.1.58.0 -l:libboost_container-mt.so.1.58.0 -l:libboost_context-mt.so.1.58.0 -l:libboost_coroutine-mt.so.1.58.0 -l:libboost_date_time-mt.so.1.58.0 -l:libboost_filesystem-mt.so.1.58.0 -l:libboost_graph-mt.so.1.58.0 -l:libboost_iostreams-mt.so.1.58.0 -l:libboost_locale-mt.so.1.58.0 -l:libboost_log_setup-mt.so.1.58.0 -l:libboost_log-mt.so.1.58.0 -l:libboost_math_c99-mt.so.1.58.0 -l:libboost_math_c99f-mt.so.1.58.0 -l:libboost_math_c99l-mt.so.1.58.0 -l:libboost_math_tr1-mt.so.1.58.0 -l:libboost_math_tr1f-mt.so.1.58.0 -l:libboost_math_tr1l-mt.so.1.58.0 -l:libboost_prg_exec_monitor-mt.so.1.58.0 -l:libboost_program_options-mt.so.1.58.0 -l:libboost_python-mt.so.1.58.0 -l:libboost_random-mt.so.1.58.0 -l:libboost_regex-mt.so.1.58.0 -l:libboost_serialization-mt.so.1.58.0 -l:libboost_signals-mt.so.1.58.0 -l:libboost_system-mt.so.1.58.0 -l:libboost_thread-mt.so.1.58.0 -l:libboost_timer-mt.so.1.58.0 -l:libboost_unit_test_framework-mt.so.1.58.0 -l:libboost_wave-mt.so.1.58.0 -l:libboost_wserialization-mt.so.1.58.0 -l:libViconDataStreamSDK_C.so -l:libViconDataStreamSDK_CPP.so -Wl,--end-group -o "a.out"