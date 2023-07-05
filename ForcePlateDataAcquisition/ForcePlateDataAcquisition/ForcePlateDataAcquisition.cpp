// ForcePlateDataAcquisition.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <windows.h>
#include <atlstr.h>
#include <iostream>
#include "AMTIUSBDeviceDefinitions.h"

int InitializeDeviceDLL(void)
{
    fmDLLInit();

    // At most 250 ms are needed to initialize the DLL according to the AMTI reference.
    Sleep(250);

    int countdown = 20;
    int deviceInitCompleteValue;
    while ((deviceInitCompleteValue = fmDLLIsDeviceInitComplete()) == 0)
    {
        Sleep(250);
        if (countdown-- <= 0)
        {
            std::cout << "Not completed initializing the DLL.\n";
        }
    }

    switch (deviceInitCompleteValue)
    {
    case 1:
        std::cout << "The DLL is initialized, no signal conditioners are present.\n";
        return -1;
        break;
    case 2:
        std::cout << "The DLL is initialized.\n";
        return -1;
        break;
    default:
        std::cout << "Unknown error is occurred.\n";
        return -1;
        break;
    }

    if (fmDLLSetupCheck() != 1)
    {
        std::cout << "Error " << fmDLLSetupCheck() << ".\n";
        return -1;
    }

    return 0;
}

int main()
{
    if (InitializeDeviceDLL() != 0)
    {
        Sleep(5000);
        exit(-1);
    }

    int deviceCount = fmDLLGetDeviceCount();
    std::cout << deviceCount << " AMTI amplifiers was found.\n";

    for (int i = 0; i < deviceCount; i++)
    {
        fmDLLSelectDeviceIndex(i);
        char* buf = new char[17];
        fmGetAmplifierSerialNumber(buf);
        std::cout << buf << ".\n";
        delete[] buf;
    }

    char len[30];
    char width[30];
    char model[30];
    char serial[30];
    char theDate[30];
    float lenWdth[2];
    float offsets[3];
    float sen[36];
    float bridgeResis[6];
    memset(len, '\0', 30);
    memset(width, '\0', 30);
    memset(model, '\0', 30);
    memset(serial, '\0', 30);
    memset(theDate, '\0', 30);

    offsets[0] = 0.0;
    offsets[1] = 0.0;
    offsets[2] = 0.0;

    for (int i = 0; i < 6; i++)
    {
        bridgeResis[i] = 0.0;
    }
    for (int i = 0; i < 36; i++)
    {
        sen[i] = 0.0;
    }

    fmDLLSelectDeviceIndex(0);
    fmGetPlatformModelNumber(model);
    std::cout << "Model: " << model << "\n";

    fmGetPlatformSerialNumber(serial);
    std::cout << "Serial: " << serial << "\n";

    fmGetPlatformDate(theDate);
    std::cout << "Date: " << theDate << "\n";

    fmGetPlatformLengthAndWidth((char*)len, (char*)width);
    std::cout << "Length: " << len << " Width: " << width;

    fmGetPlatformXYZOffsets(offsets);
    std::cout << "Offsets: X: " << offsets[0] << "Y: " << offsets[1] << "Z: " << offsets[2];

    // fmGetPlatformBridgeResistance(bridgeResis);
    // fmGetInvertedSensitivityMatrix(sen);

    ////////////////////////////////////// Get Data //////////////////////////////////////
    while (true)
    {
        fmDLLSetDataFormat(0);
        float* ptr;
        int ret, i;
        CString str, dum;
        ret = fmDLLTransferFloatData((float*&)ptr);
        if (ret == 0)
        {
            return 0;
        }
        str = "";
        dum = "";
        for (i = 0; i < 16; i++)
        {
            dum.Format(_T("%6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, % 6.3f \r\n"), ptr[0], ptr[1], ptr[2], ptr[3], ptr[4], ptr[5], ptr[6], ptr[7]);
            ptr += 8;
            str += dum;
        }
    }

    Sleep(5000);
    return 0;
}


// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
