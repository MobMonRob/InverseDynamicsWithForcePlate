#ifndef _AMTIUSBDeviceDefinitions_h_
//
//      AMTIUSBDeviceDefinitions.h
//
//      Prototype definitions for the AMTI SDK user interface
//


//      Macro for DLL exports in Win32, replaces Win16 __export
#define DLLExport _declspec(dllexport)

#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */

// ************************************************
//      DLL Initialization Functions
// ************************************************

int     DLLExport fmDLLGetVersionID(void);
void    DLLExport fmDLLInit(void);
int     DLLExport fmDLLIsDeviceInitComplete(void);
int     DLLExport fmDLLSetupCheck(void);
void    DLLExport fmDLLSetUSBPacketSize(int size);
int     DLLExport fmDLLGetDeviceCount(void);
void    DLLExport fmDLLSelectDeviceIndex(int index);
int     DLLExport fmDLLGetDeviceIndex(void);
int     DLLExport fmDLLSaveConfiguration(void);
int     DLLExport fmDLLShutDown(void);
int     DLLExport fmIsDLLShutDown(void);


// ************************************************
//      Data Collection Functions
// ************************************************

void    DLLExport fmBroadcastRunMode(int mode);
int     DLLExport fmDLLGetRunMode(void);
int     DLLExport fmGetRunMode(void);
void    DLLExport fmBroadcastGenlock(int mode);
int     DLLExport fmDLLGetGenlock(void); 
void    DLLExport fmBroadcastAcquisitionRate(int rate);
int     DLLExport fmDLLGetAcquisitionRate(void);
int     DLLExport fmGetAcquisitionRate(void);
void    DLLExport fmBroadcastStart(void);
void    DLLExport fmBroadcastStop(void);
void    DLLExport fmBroadcastZero(void);
void    DLLExport fmDLLPostDataReadyMessages(int mode); 
void    DLLExport fmDLLPostWindowMessages(HWND handle);
void    DLLExport fmDLLPostUserThreadMessages(unsigned int threadID);
void    DLLExport fmDLLSetDataFormat(int format);
int     DLLExport fmDLLTransferFloatData(float *&data);
int     DLLExport fmDLLGetTheFloatDataLBVStyle(float *data, int size);


// ************************************************
//      Apply and Save Functions
// ************************************************

void    DLLExport fmBroadcastResetSoftware(void);
void    DLLExport fmResetSoftware(void);
void    DLLExport fmBroadcastSave(void);
void    DLLExport fmSave(void);
void    DLLExport fmApplyLimited(void);


// ************************************************
//      Signal Conditioner Configuration Functions
// ************************************************
void    DLLExport fmSetCurrentGains(long *ldata);
void    DLLExport fmGetCurrentGains(long *ldata);
void    DLLExport fmSetCurrentExcitations(long *ldata);
void    DLLExport fmGetCurrentExcitations(long *ldata);
void    DLLExport fmSetChannelOffsetsTable(float *fdata);
void    DLLExport fmGetChannelOffsetsTable(float *fdata);
void    DLLExport fmSetCableLength(float fdata);
float   DLLExport fmGetCableLength(void);
void    DLLExport fmSetMatrixMode(long mode);
long    DLLExport fmGetMatrixMode(void);
void    DLLExport fmSetPlatformRotation(float rotation);
float   DLLExport fmGetPlatformRotation(void);


// ************************************************
//      Mechanical Limit Functions
// ************************************************

void    DLLExport fmUpdateMechanicalMaxAndMin(void);
int     DLLExport fmGetMechanicalMaxAndMin(float *data);
void    DLLExport fmUpdateAnalogMaxAndMin(void);
int     DLLExport fmGetAnalogMaxAndMin(float *data);


// ************************************************
//      Platform Ordering Functions
// ************************************************
                                                        
void    DLLExport fmDLLSetPlatformOrder(int *device_map);
void    DLLExport fmBroadcastPlatformOrderingThreshold(float value);
void    DLLExport fmDLLStartPlatformOrdering(void);
int     DLLExport fmDLLIsPlatformOrderingComplete(void);
void    DLLExport fmDLLCancelPlatformOrdering(void);


// ************************************************
//      Signal Conditioner Calibration Functions
// ************************************************

long    DLLExport fmGetProductType(void);
void    DLLExport fmGetAmplifierModelNumber(char *Cdata);
void    DLLExport fmGetAmplifierSerialNumber(char *Cdata);
void    DLLExport fmGetAmplifierFirmwareVersion(char *Cdata);
void    DLLExport fmGetAmplifierDate(char *Cdata);
void    DLLExport fmGetGainTable(float *data);                                                  
void    DLLExport fmGetExcitationTable(float *data);
void    DLLExport fmGetDACGainsTable(float *data);
void    DLLExport fmGetDACOffsetTable(float *data);
void    DLLExport fmSetDACSensitivityTable(float *data);
void    DLLExport fmGetDACSensitivities(float *data);
float   DLLExport fmGetADRef(void);


// ************************************************
//      Platform Calibration Functions
// ************************************************

void    DLLExport fmSetPlatformDate(char *Cdata);
void    DLLExport fmGetPlatformDate(char *Cdata);
void    DLLExport fmSetPlatformModelNumber(char *Cdata);
void    DLLExport fmGetPlatformModelNumber(char *Cdata);
void    DLLExport fmSetPlatformSerialNumber(char *Cdata);
void    DLLExport fmGetPlatformSerialNumber(char *Cdata);
void    DLLExport fmSetPlatformLengthAndWidth(char *Ldata, char *Wdata);
void    DLLExport fmGetPlatformLengthAndWidth(char *Ldata, char *Wdata);
void    DLLExport fmSetPlatformXYZOffsets(float *data);
void    DLLExport fmGetPlatformXYZOffsets(float *data);
void    DLLExport fmSetPlatformCapacity(float *data);
void    DLLExport fmGetPlatformCapacity(float *data);
void    DLLExport fmSetPlatformBridgeResistance(float *data);
void    DLLExport fmGetPlatformBridgeResistance(float *data);
void    DLLExport fmSetInvertedSensitivityMatrix(float *data);
void    DLLExport fmGetInvertedSensitivityMatrix(float *data);
void    DLLExport fmSetPlatformXYZExtensions(float *data);
void    DLLExport fmGetPlatformXYZExtensions(float *data);

// ************************************************
//      Signal Conditioner Hardware Functions
// ************************************************

void    DLLExport fmSetBlink(void);
void    DLLExport fmResetHardware(void);
void    DLLExport fmBroadcastResetUSB( void );


// ************************************************
//      Optima Support Functions
// ************************************************

long    DLLExport fmBroadcastCheckOptima(long *data);
long    DLLExport fmOptimaGetStatus(void);
long    DLLExport fmOptimaDownloadCalFile(BOOL flag);
long    DLLExport fmIsOptimaDownloadComplete(void);


#ifdef __cplusplus
}
#endif

#define  _AMTIUSBDeviceDefinitions_h_
#endif

