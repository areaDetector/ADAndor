/* #pragma hdrstop */

#define SHAMROCK_COMMUNICATION_ERROR 20201
#define SHAMROCK_SUCCESS 20202
#define SHAMROCK_P1INVALID 20266
#define SHAMROCK_P2INVALID 20267
#define SHAMROCK_P3INVALID 20268
#define SHAMROCK_P4INVALID 20269
#define SHAMROCK_P5INVALID 20270
#define SHAMROCK_NOT_INITIALIZED 20275
#define SHAMROCK_NOT_AVAILABLE 20292

#define SHAMROCK_ACCESSORYMIN 1
#define SHAMROCK_ACCESSORYMAX 2
#define SHAMROCK_FILTERMIN 1
#define SHAMROCK_FILTERMAX 6
#define SHAMROCK_TURRETMIN 1
#define SHAMROCK_TURRETMAX 3
#define SHAMROCK_GRATINGMIN 1
#define SHAMROCK_SLITWIDTHMIN 10
#define SHAMROCK_SLITWIDTHMAX 2500
#define SHAMROCK_I24SLITWIDTHMAX 24000
#define SHAMROCK_SHUTTERMODEMIN 0
#define SHAMROCK_SHUTTERMODEMAX 1
#define SHAMROCK_DET_OFFSET_MIN -240000
#define SHAMROCK_DET_OFFSET_MAX 240000
#define SHAMROCK_GRAT_OFFSET_MIN -20000
#define SHAMROCK_GRAT_OFFSET_MAX 20000

#define SHAMROCK_SLIT_INDEX_MIN    1
#define SHAMROCK_SLIT_INDEX_MAX    4

#define SHAMROCK_INPUT_SLIT_SIDE   1
#define SHAMROCK_INPUT_SLIT_DIRECT  2
#define SHAMROCK_OUTPUT_SLIT_SIDE  3
#define SHAMROCK_OUTPUT_SLIT_DIRECT 4

#define SHAMROCK_FLIPPER_INDEX_MIN    1
#define SHAMROCK_FLIPPER_INDEX_MAX    2
#define SHAMROCK_PORTMIN 0
#define SHAMROCK_PORTMAX 1

#define SHAMROCK_INPUT_FLIPPER   1
#define SHAMROCK_OUTPUT_FLIPPER  2
#define SHAMROCK_DIRECT_PORT  0
#define SHAMROCK_SIDE_PORT    1

#define SHAMROCK_ERRORLENGTH 64

#ifndef __linux__
 #include "windows.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

#ifdef linux
  #define WINAPI
#endif

//sdkbasic functions
unsigned int WINAPI ShamrockInitialize(char * IniPath);
unsigned int WINAPI ShamrockClose(void);
unsigned int WINAPI ShamrockGetNumberDevices(int *nodevices);
unsigned int WINAPI ShamrockGetFunctionReturnDescription(int error,char *description, int MaxDescStrLen);
//sdkeeprom functions
unsigned int WINAPI ShamrockGetSerialNumber(int device,char *serial);
unsigned int WINAPI ShamrockEepromGetOpticalParams(int device,float *FocalLength,float *AngularDeviation,float *FocalTilt);
//sdkgrating functions
unsigned int WINAPI ShamrockSetGrating(int device,int grating);
unsigned int WINAPI ShamrockGetGrating(int device,int *grating);
unsigned int WINAPI ShamrockWavelengthReset(int device);
unsigned int WINAPI ShamrockGetNumberGratings(int device,int *noGratings);
unsigned int WINAPI ShamrockGetGratingInfo(int device,int Grating, float *Lines,  char* Blaze, int *Home, int *Offset);
unsigned int WINAPI ShamrockSetDetectorOffset(int device,int offset);
unsigned int WINAPI ShamrockGetDetectorOffset(int device,int *offset);
unsigned int WINAPI ShamrockSetDetectorOffsetPort2(int device,int offset);
unsigned int WINAPI ShamrockGetDetectorOffsetPort2(int device,int *offset);
unsigned int WINAPI ShamrockSetDetectorOffsetEx(int device, int entrancePort, int exitPort, int offset);
unsigned int WINAPI ShamrockGetDetectorOffsetEx(int device, int entrancePort, int exitPort, int *offset);
unsigned int WINAPI ShamrockSetGratingOffset(int device,int Grating, int offset);
unsigned int WINAPI ShamrockGetGratingOffset(int device,int Grating, int *offset);
unsigned int WINAPI ShamrockGratingIsPresent(int device,int *present);
unsigned int WINAPI ShamrockSetTurret(int device,int Turret);
unsigned int WINAPI ShamrockGetTurret(int device,int *Turret);
//sdkwavelength functions
unsigned int WINAPI ShamrockSetWavelength(int device, float wavelength);
unsigned int WINAPI ShamrockGetWavelength(int device, float *wavelength);
unsigned int WINAPI ShamrockGotoZeroOrder(int device);
unsigned int WINAPI ShamrockAtZeroOrder(int device, int *atZeroOrder);
unsigned int WINAPI ShamrockGetWavelengthLimits(int device, int Grating, float *Min, float *Max);
unsigned int WINAPI ShamrockWavelengthIsPresent(int device,int *present);
//sdkslit functions

// New Slit Functions
unsigned int WINAPI ShamrockSetAutoSlitWidth(int device, int index, float width);
unsigned int WINAPI ShamrockGetAutoSlitWidth(int device, int index, float *width);
unsigned int WINAPI ShamrockAutoSlitReset(int device, int index);
unsigned int WINAPI ShamrockAutoSlitIsPresent(int device, int index, int *present);
unsigned int WINAPI ShamrockSetAutoSlitCoefficients(int device, int index, int x1, int y1, int x2, int y2);
unsigned int WINAPI ShamrockGetAutoSlitCoefficients(int device, int index, int &x1, int &y1, int &x2, int &y2);

///// Deprecated Slit Functions
// Deprecated Input Slit Functions
unsigned int WINAPI ShamrockSetSlit(int device,float width);
unsigned int WINAPI ShamrockGetSlit(int device,float *width);
unsigned int WINAPI ShamrockSlitReset(int device);
unsigned int WINAPI ShamrockSlitIsPresent(int device,int *present);
unsigned int WINAPI ShamrockSetSlitCoefficients(int device, int x1, int y1, int x2, int y2);
unsigned int WINAPI ShamrockGetSlitCoefficients(int device, int &x1, int &y1, int &x2, int &y2);

// Deprecated Ouput Slit functions
unsigned int WINAPI ShamrockSetOutputSlit(int device,float width);
unsigned int WINAPI ShamrockGetOutputSlit(int device,float *width);
unsigned int WINAPI ShamrockOutputSlitReset(int device);
unsigned int WINAPI ShamrockOutputSlitIsPresent(int device,int *present);
/////

//sdkshutter functions
unsigned int WINAPI ShamrockSetShutter(int device,int mode);
unsigned int WINAPI ShamrockGetShutter(int device, int *mode);
unsigned int WINAPI ShamrockIsModePossible(int device,int mode,int *possible);
unsigned int WINAPI ShamrockShutterIsPresent(int device,int *present);
//sdkfilter functions
unsigned int WINAPI ShamrockSetFilter(int device,int filter);
unsigned int WINAPI ShamrockGetFilter(int device,int *filter);
unsigned int WINAPI ShamrockGetFilterInfo(int device,int Filter, char* Info);
unsigned int WINAPI ShamrockSetFilterInfo(int device,int Filter, char* Info);
unsigned int WINAPI ShamrockFilterReset(int device);
unsigned int WINAPI ShamrockFilterIsPresent(int device,int *present);

//sdkflipper functions

// New flipper functions
unsigned int WINAPI ShamrockSetFlipperMirror(int device, int flipper, int port);
unsigned int WINAPI ShamrockGetFlipperMirror(int device, int flipper, int * port);
unsigned int WINAPI ShamrockFlipperMirrorReset(int device, int flipper);
unsigned int WINAPI ShamrockFlipperMirrorIsPresent(int device, int flipper, int *present);
unsigned int WINAPI ShamrockGetCCDLimits(int device, int port, float *Low, float *High);

// Deprecated
unsigned int WINAPI ShamrockSetPort(int device,int port);
unsigned int WINAPI ShamrockGetPort(int device, int*port);
unsigned int WINAPI ShamrockFlipperReset(int device);
unsigned int WINAPI ShamrockFlipperIsPresent(int device,int *present);

//sdkaccessory functions
unsigned int WINAPI ShamrockSetAccessory(int device,int Accessory, int State);
unsigned int WINAPI ShamrockGetAccessoryState(int device,int Accessory, int *state);
unsigned int WINAPI ShamrockAccessoryIsPresent(int device,int *present);

//sdkshutter functions
unsigned int WINAPI ShamrockSetFocusMirror(int device, int focus);
unsigned int WINAPI ShamrockGetFocusMirror(int device, int *focus);
unsigned int WINAPI ShamrockGetFocusMirrorMaxSteps(int device, int *steps);
unsigned int WINAPI ShamrockFocusMirrorReset(int device);
unsigned int WINAPI ShamrockFocusMirrorIsPresent(int device, int *present);

//sdkcalibration functions
unsigned int WINAPI ShamrockSetPixelWidth(int device, float Width);
unsigned int WINAPI ShamrockSetNumberPixels(int device, int NumberPixels);
unsigned int WINAPI ShamrockGetPixelWidth(int device, float* Width);
unsigned int WINAPI ShamrockGetNumberPixels(int device, int* NumberPixels);
unsigned int WINAPI ShamrockGetCalibration(int device, float* CalibrationValues, int NumberPixels);
unsigned int WINAPI ShamrockGetPixelCalibrationCoefficients(int device, float* A, float* B, float* C, float* D);
#ifdef __cplusplus
}
#endif
