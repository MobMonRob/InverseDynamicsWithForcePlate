#include "DataStreamClient.h"

#include <iostream>
#include "ForcePlateDataAcquisition.hpp"

using namespace Acquisition;

int main()
{
    // TODO: Add several filters, for example moving average filter.
    try
    {
        ForcePlateDataAcquisition forcePlateDataAcquisition;

        while (true)
        {
            forcePlateDataAcquisition.grabDirect();
        }
    }
    catch (const std::exception &e)
    {
        std::cout << e.what();
    }

    return 0;
}

// Compile for Vicon SDK 1.10
// g++ -cpp "./Main.cpp" "./DataStreamClientFacade.cpp" "./ForcePlateDataAcquisition.cpp" -I"./ViconLib1.10" -Wl,--start-group -L"./ViconLib1.10" -l:libboost_atomic-mt.so.1.58.0 -l:libboost_chrono-mt.so.1.58.0 -l:libboost_container-mt.so.1.58.0 -l:libboost_context-mt.so.1.58.0 -l:libboost_coroutine-mt.so.1.58.0 -l:libboost_date_time-mt.so.1.58.0 -l:libboost_filesystem-mt.so.1.58.0 -l:libboost_graph-mt.so.1.58.0 -l:libboost_iostreams-mt.so.1.58.0 -l:libboost_locale-mt.so.1.58.0 -l:libboost_log_setup-mt.so.1.58.0 -l:libboost_log-mt.so.1.58.0 -l:libboost_math_c99-mt.so.1.58.0 -l:libboost_math_c99f-mt.so.1.58.0 -l:libboost_math_c99l-mt.so.1.58.0 -l:libboost_math_tr1-mt.so.1.58.0 -l:libboost_math_tr1f-mt.so.1.58.0 -l:libboost_math_tr1l-mt.so.1.58.0 -l:libboost_prg_exec_monitor-mt.so.1.58.0 -l:libboost_program_options-mt.so.1.58.0 -l:libboost_python-mt.so.1.58.0 -l:libboost_random-mt.so.1.58.0 -l:libboost_regex-mt.so.1.58.0 -l:libboost_serialization-mt.so.1.58.0 -l:libboost_signals-mt.so.1.58.0 -l:libboost_system-mt.so.1.58.0 -l:libboost_thread-mt.so.1.58.0 -l:libboost_timer-mt.so.1.58.0 -l:libboost_unit_test_framework-mt.so.1.58.0 -l:libboost_wave-mt.so.1.58.0 -l:libboost_wserialization-mt.so.1.58.0 -l:libViconDataStreamSDK_C.so -l:libViconDataStreamSDK_CPP.so -Wl,--end-group -Wl,-rpath,'$ORIGIN/ViconLib1.10/' -o "a.out"

// Compile for Vicon SDK 1.13
// g++ -cpp "./Main.cpp" "./DataStreamClientFacade.cpp" "./ForcePlateDataAcquisition.cpp" -I"./ViconLib1.12" -Wl,--start-group -L"./ViconLib1.12" -l:libboost_atomic-mt-x64.so.1.75.0 -l:libboost_chrono-mt-x64.so.1.75.0 -l:libboost_container-mt-x64.so.1.75.0 -l:libboost_context-mt-x64.so.1.75.0 -l:libboost_coroutine-mt-x64.so.1.75.0 -l:libboost_date_time-mt-x64.so.1.75.0 -l:libboost_filesystem-mt-x64.so.1.75.0 -l:libboost_graph-mt-x64.so.1.75.0 -l:libboost_locale-mt-x64.so.1.75.0 -l:libboost_log_setup-mt-x64.so.1.75.0 -l:libboost_log-mt-x64.so.1.75.0 -l:libboost_math_c99-mt-x64.so.1.75.0 -l:libboost_math_c99f-mt-x64.so.1.75.0 -l:libboost_math_c99l-mt-x64.so.1.75.0 -l:libboost_math_tr1-mt-x64.so.1.75.0 -l:libboost_math_tr1f-mt-x64.so.1.75.0 -l:libboost_math_tr1l-mt-x64.so.1.75.0 -l:libboost_prg_exec_monitor-mt-x64.so.1.75.0 -l:libboost_program_options-mt-x64.so.1.75.0 -l:libboost_random-mt-x64.so.1.75.0 -l:libboost_regex-mt-x64.so.1.75.0 -l:libboost_serialization-mt-x64.so.1.75.0 -l:libboost_system-mt-x64.so.1.75.0 -l:libboost_thread-mt-x64.so.1.75.0 -l:libboost_timer-mt-x64.so.1.75.0 -l:libboost_unit_test_framework-mt-x64.so.1.75.0 -l:libboost_wave-mt-x64.so.1.75.0 -l:libboost_wserialization-mt-x64.so.1.75.0 -l:libViconDataStreamSDK_C.so -l:libViconDataStreamSDK_CPP.so -Wl,--end-group -Wl,-rpath,'$ORIGIN/ViconLib1.12/' -o "a.out"
