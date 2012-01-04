
// Linux C Interface
#if !defined(__atmcdLXd_h)
#define __atmcdLXd_h

#ifdef __cplusplus
extern "C" {
#endif

#define at_u16 unsigned short
#ifdef _LP64
#define at_32 int
#define at_u32 unsigned int
#else
#define at_32 long
#define at_u32 unsigned long
#endif

#define at_64 long long
#define at_u64 unsigned long long 
		
// ===================================
// Version Information Definitions
// ===================================

//Version Information Enumeration - NOTE: Increment the count constant below when
//                                       this structure is extended
// Using large numbers to force size to an integer
typedef enum {
  // Using large numbers to force size to an integer
  AT_SDKVersion = 0x40000000,
		AT_DeviceDriverVersion = 0x40000001
} AT_VersionInfoId;
		
// Count of the number of elements in the Version Information Enumeration
// NOTE: Increment when extending enumeration
#define AT_NoOfVersionInfoIds 2

// Minimum recommended length of the Version Info buffer parameter
#define AT_VERSION_INFO_LEN 80
// Minimum recommended length of the Controller Card Model buffer parameter
#define AT_CONTROLLER_CARD_MODEL_LEN 80
// ===================================
		
// ===================================
// DDG Lite Definitions
// ===================================

//Channel enumeration      
typedef enum {
  // Using large numbers to force size to an integer
  AT_DDGLite_ChannelA = 0x40000000,
		AT_DDGLite_ChannelB = 0x40000001,
		AT_DDGLite_ChannelC = 0x40000002
} AT_DDGLiteChannelId;
		
// Control byte flags
#define AT_DDGLite_ControlBit_GlobalEnable   0x01

#define AT_DDGLite_ControlBit_ChannelEnable  0x01
#define AT_DDGLite_ControlBit_FreeRun        0x02
#define AT_DDGLite_ControlBit_DisableOnFrame 0x04
#define AT_DDGLite_ControlBit_RestartOnFire  0x08
#define AT_DDGLite_ControlBit_Invert         0x10
#define AT_DDGLite_ControlBit_EnableOnFire   0x20
// ===================================
		
// ===================================
// USB iStar Definitions
// ===================================

#define AT_DDG_POLARITY_POSITIVE  0
#define AT_DDG_POLARITY_NEGATIVE  1
#define AT_DDG_TERMINATION_50OHMS 0
#define AT_DDG_TERMINATION_HIGHZ  1

#define AT_STEPMODE_CONSTANT      0
#define AT_STEPMODE_EXPONENTIAL   1
#define AT_STEPMODE_LOGARITHMIC   2
#define AT_STEPMODE_LINEAR        3
#define AT_STEPMODE_OFF           100

#define AT_GATEMODE_FIRE_AND_GATE 0
#define AT_GATEMODE_FIRE_ONLY     1
#define AT_GATEMODE_GATE_ONLY     2
#define AT_GATEMODE_CW_ON         3
#define AT_GATEMODE_CW_OFF        4
#define AT_GATEMODE_DDG           5
// ===================================
		
typedef struct ANDORCAPS
{
		at_u32 ulSize;
		at_u32 ulAcqModes;
		at_u32 ulReadModes;
		at_u32 ulTriggerModes;
		at_u32 ulCameraType;
		at_u32 ulPixelMode;
		at_u32 ulSetFunctions;
		at_u32 ulGetFunctions;
		at_u32 ulFeatures;
		at_u32 ulPCICard;
		at_u32 ulEMGainCapability;
		at_u32 ulFTReadModes;
} AndorCapabilities;
	

typedef struct COLORDEMOSAICINFO
{
		int iX;
		int iY;
		int iAlgorithm;
		int iXPhase;
		int iYPhase;
		int iBackground;
} ColorDemosaicInfo;
	

typedef struct WHITEBALANCEINFO
{
		int iSize;
		int iX;
		int iY;
		int iAlgorithm;
		int iROI_left;
		int iROI_right;
		int iROI_top;
		int iROI_bottom;
		int iOperation;
} WhiteBalanceInfo;
	

unsigned int AbortAcquisition();
unsigned int CancelWait();
unsigned int CoolerOFF();
unsigned int CoolerON();
unsigned int DemosaicImage(unsigned short * grey, unsigned short * red, unsigned short * green, unsigned short * blue, ColorDemosaicInfo * info);
unsigned int EnableKeepCleans(int iMode);
unsigned int FreeInternalMemory();
unsigned int GetAcquiredData(at_32 * arr, at_u32 size);
unsigned int GetAcquiredData16(unsigned short * arr, at_u32 size);
unsigned int GetAcquiredFloatData(float * arr, at_u32 size);
unsigned int GetAcquisitionProgress(at_32 * acc, at_32 * series);
unsigned int GetAcquisitionTimings(float * exposure, float * accumulate, float * kinetic);
unsigned int GetAdjustedRingExposureTimes(int inumTimes, float * fptimes);
unsigned int GetAllDMAData(at_32 * arr, at_u32 size);
unsigned int GetAmpDesc(int index, char * name, int length);
unsigned int GetAmpMaxSpeed(int index, float * speed);
unsigned int GetAvailableCameras(at_32 * totalCameras);
unsigned int GetBackground(at_32 * arr, at_u32 size);
unsigned int GetBitDepth(int channel, int * depth);
unsigned int GetCameraEventStatus(at_u32 * camStatus);
unsigned int GetCameraHandle(at_32 cameraIndex, at_32 * cameraHandle);
unsigned int GetCameraInformation(int index, at_32 * information);
unsigned int GetCameraSerialNumber(int * number);
unsigned int GetCapabilities(AndorCapabilities * caps);
unsigned int GetControllerCardModel(char * controllerCardModel);
unsigned int GetCurrentCamera(at_32 * cameraHandle);
unsigned int GetCYMGShift(int * iXshift, int * iYShift);
unsigned int GetDDGExternalOutputEnabled(at_u32 uiIndex, at_u32 * puiEnabled);
unsigned int GetDDGExternalOutputPolarity(at_u32 uiIndex, at_u32 * puiPolarity);
unsigned int GetDDGExternalOutputTime(at_u32 uiIndex, at_u64 * puiDelay, at_u64 * puiWidth);
unsigned int GetDDGGateTime(at_u64 * puiDelay, at_u64 * puiWidth);
unsigned int GetDDGInsertionDelay(int * piState);
unsigned int GetDDGIntelligate(int * piState);
unsigned int GetDDGIOC(int * state);
unsigned int GetDDGIOCFrequency(double * frequency);
unsigned int GetDDGIOCNumber(unsigned long * numberPulses);
unsigned int GetDDGIOCNumberRequested(at_u32 * pulses);
unsigned int GetDDGIOCPeriod(at_u64 * period);
unsigned int GetDDGIOCPulses(int * pulses);

// DDG Lite functions
unsigned int GetDDGLiteGlobalControlByte(unsigned char * control);
unsigned int GetDDGLiteControlByte(AT_DDGLiteChannelId channel, unsigned char * control);
unsigned int GetDDGLiteInitialDelay(AT_DDGLiteChannelId channel, float * fDelay);
unsigned int GetDDGLitePulseWidth(AT_DDGLiteChannelId channel, float * fWidth);
unsigned int GetDDGLiteInterPulseDelay(AT_DDGLiteChannelId channel, float * fDelay);
unsigned int GetDDGLitePulsesPerExposure(AT_DDGLiteChannelId channel, at_u32 * ui32Pulses);

unsigned int GetDDGPulse(double wid, double resolution, double * Delay, double * Width);
unsigned int GetDDGStepCoefficients(at_u32 mode, double * p1, double * p2);
unsigned int GetDDGStepMode(at_u32 * mode);
unsigned int GetDetector(int * xpixels, int * ypixels);
unsigned int GetDICameraInfo(void * info);
unsigned int GetEMCCDGain(int * gain);
unsigned int GetEMGainRange(int * low, int * high);
unsigned int GetExternalTriggerTermination(at_u32 * puiTermination);
unsigned int GetFastestRecommendedVSSpeed(int * index, float * speed);
unsigned int GetFIFOUsage(int * FIFOusage);
unsigned int GetFilterMode(int * mode);
unsigned int GetFKExposureTime(float * time);
unsigned int GetFKVShiftSpeed(int index, int * speed);
unsigned int GetFKVShiftSpeedF(int index, float * speed);
unsigned int GetGateMode(int * piGatemode);
unsigned int GetHardwareVersion(unsigned int * PCB, unsigned int * Decode, unsigned int * dummy1, unsigned int * dummy2, unsigned int * CameraFirmwareVersion, unsigned int * CameraFirmwareBuild);
unsigned int GetHeadModel(char * name);
unsigned int GetHorizontalSpeed(int index, int * speed);
unsigned int GetHSSpeed(int channel, int typ, int index, float * speed);
unsigned int GetHVflag(int * bFlag);
unsigned int GetID(int devNum, int * id);
unsigned int GetImageFlip(int * iHFlip, int * iVFlip);
unsigned int GetImageRotate(int * iRotate);
unsigned int GetImages(at_32 first, at_32 last, at_32 * arr, at_u32 size, at_32 * validfirst, at_32 * validlast);
unsigned int GetImages16(at_32 first, at_32 last, unsigned short * arr, at_u32 size, at_32 * validfirst, at_32 * validlast);
unsigned int GetImagesPerDMA(at_u32 * images);
unsigned int GetIRQ(int * IRQ);
unsigned int GetKeepCleanTime(float * KeepCleanTime);
unsigned int GetMaximumBinning(int ReadMode, int HorzVert, int * MaxBinning);
unsigned int GetMaximumExposure(float * MaxExp);
unsigned int GetMCPGain(int * piGain);
unsigned int GetMCPGainRange(int * iLow, int * iHigh);
unsigned int GetMCPGainTable(int iNum, int * piGain, float * pfPhotoepc);
unsigned int GetMCPVoltage(int * iVoltage);
unsigned int GetMinimumImageLength(int * MinImageLength);
unsigned int GetMinimumNumberInSeries(int * number);
unsigned int GetMostRecentColorImage16(at_u32 size, int algorithm, unsigned short * red, unsigned short * green, unsigned short * blue);
unsigned int GetMostRecentImage(at_32 * arr, at_u32 size);
unsigned int GetMostRecentImage16(unsigned short * arr, at_u32 size);
// unsigned int GetMSTimingsData(short * TimeOfStart, float * pfDifferences, int inoOfImages);
// unsigned int GetMetaDataInfo(short * TimeOfStart, float * pfTimeFromStart, unsigned int index);
// unsigned int GetMSTimingsEnabled();
unsigned int GetNewData(at_32 * arr, at_u32 size);
unsigned int GetNewData16(unsigned short * arr, at_u32 size);
unsigned int GetNewData8(unsigned char * arr, at_u32 size);
unsigned int GetNewFloatData(float * arr, at_u32 size);
unsigned int GetNumberADChannels(int * channels);
unsigned int GetNumberAmp(int * amp);
unsigned int GetNumberAvailableImages(at_32 * first, at_32 * last);
unsigned int GetNumberDDGExternalOutputs(at_u32 * puiCount);
unsigned int GetNumberDevices(int * numDevs);
unsigned int GetNumberFKVShiftSpeeds(int * number);
unsigned int GetNumberHorizontalSpeeds(int * number);
unsigned int GetNumberHSSpeeds(int channel, int typ, int * speeds);
unsigned int GetNumberNewImages(at_32 * first, at_32 * last);
unsigned int GetNumberPreAmpGains(int * noGains);
unsigned int GetNumberRingExposureTimes(int * ipnumTimes);
unsigned int GetNumberIO(int * iNumber);
unsigned int GetNumberVerticalSpeeds(int * number);
unsigned int GetNumberVSAmplitudes(int * number);
unsigned int GetNumberVSSpeeds(int * speeds);
unsigned int GetOldestImage(at_32 * arr, at_u32 size);
unsigned int GetOldestImage16(unsigned short * arr, at_u32 size);
unsigned int GetPhosphorStatus(int * piFlag);
unsigned int GetPhysicalDMAAddress(at_u32 * Address1, at_u32 * Address2);
unsigned int GetPixelSize(float * xSize, float * ySize);
unsigned int GetPreAmpGain(int index, float * gain);
unsigned int GetReadOutTime(float * ReadOutTime);
unsigned int GetRegisterDump(int * mode);
unsigned int GetRingExposureRange(float * fpMin, float * fpMax);
unsigned int GetSizeOfCircularBuffer(at_32 * index);
unsigned int GetSlotBusDeviceFunction(at_u32 * dwslot, at_u32 * dwBus, at_u32 * dwDevice, at_u32 * dwFunction);
unsigned int GetSoftwareVersion(unsigned int * eprom, unsigned int * coffile, unsigned int * vxdrev, unsigned int * vxdver, unsigned int * dllrev, unsigned int * dllver);
unsigned int GetSpoolProgress(at_32 * index);
unsigned int GetStatus(int * status);
unsigned int GetTemperature(int * temperature);
unsigned int GetTemperatureF(float * temperature);
unsigned int GetTemperatureRange(int * mintemp, int * maxtemp);
unsigned int GetTemperatureStatus(float * SensorTemp, float * TargetTemp, float * AmbientTemp, float * CoolerVolts);
unsigned int GetTotalNumberImagesAcquired(at_32 * index);
unsigned int GetIODirection(int index, int * iDirection);
unsigned int GetIOLevel(int index, int * iLevel);
unsigned int GetVersionInfo(AT_VersionInfoId arr, char * szVersionInfo, at_u32 ui32BufferLen);
unsigned int GetVerticalSpeed(int index, int * speed);
unsigned int GetVirtualDMAAddress(void ** Address1, void ** Address2);
unsigned int GetVSSpeed(int index, float * speed);
unsigned int GPIBReceive(int id, short address, char * text, int size);
unsigned int GPIBSend(int id, short address, char * text);
unsigned int I2CBurstRead(unsigned char i2cAddress, at_32 nBytes, unsigned char * data);
unsigned int I2CBurstWrite(unsigned char i2cAddress, at_32 nBytes, unsigned char * data);
unsigned int I2CRead(unsigned char deviceID, unsigned char intAddress, unsigned char * pdata);
unsigned int I2CReset();
unsigned int I2CWrite(unsigned char deviceID, unsigned char intAddress, unsigned char data);
unsigned int IdAndorDll();
unsigned int InAuxPort(int port, int * state);
unsigned int Initialize(char * dir);
unsigned int InitializeDevice(char * dir);
unsigned int IsAmplifierAvailable(int iamp);
unsigned int IsCoolerOn(int * iCoolerStatus);
unsigned int IsInternalMechanicalShutter(int * InternalShutter);
unsigned int IsPreAmpGainAvailable(int channel, int amplifier, int index, int pa, int * status);
unsigned int IsTriggerModeAvailable(int iTriggerMode);
unsigned int Merge(const at_32 * arr, at_32 nOrder, at_32 nPoint, at_32 nPixel, float * coeff, at_32 fit, at_32 hbin, at_32 * output, float * start, float * step_Renamed);
unsigned int OutAuxPort(int port, int state);
unsigned int PrepareAcquisition();
unsigned int SaveAsBmp(char * path, char * palette, at_32 ymin, at_32 ymax);
unsigned int SaveAsCommentedSif(char * path, char * comment);
unsigned int SaveAsEDF(char * szPath, int iMode);
unsigned int SaveAsFITS(char * szFileTitle, int typ);
unsigned int SaveAsRaw(char * szFileTitle, int typ);
unsigned int SaveAsSif(char * path);
// unsigned int SaveAsSPC(char * path);
unsigned int SaveAsTiff(char * path, char * palette, int position, int typ);
unsigned int SaveAsTiffEx(char * path, char * palette, int position, int typ, int mode);
unsigned int SaveEEPROMToFile(char * cFileName);
unsigned int SaveToClipBoard(char * palette);
unsigned int SelectDevice(int devNum);
unsigned int SendSoftwareTrigger();
unsigned int SetAccumulationCycleTime(float time);
// unsigned int SetAcqStatusEvent(HANDLE statusEvent);
unsigned int SetAcquisitionMode(int mode);
unsigned int SetAcquisitionType(int typ);
unsigned int SetADChannel(int channel);
unsigned int SetAdvancedTriggerModeState(int iState);
unsigned int SetBackground(at_32 * arr, at_u32 size);
unsigned int SetBaselineClamp(int state);
unsigned int SetBaselineOffset(int offset);
unsigned int SetCameraStatusEnable(unsigned long Enable);
unsigned int SetComplexImage(int numAreas, int * areas);
unsigned int SetCoolerMode(int mode);
unsigned int SetCropMode(int active, int cropHeight, int reserved);
unsigned int SetCurrentCamera(at_32 cameraHandle);
unsigned int SetCustomTrackHBin(int bin);
unsigned int SetDataType(int typ);
unsigned int SetDACOutput(int iOption, int iResolution, int iValue);
unsigned int SetDACOutputScale(int iScale);
unsigned int SetDDGAddress(unsigned char t0, unsigned char t1, unsigned char t2, unsigned char t3, unsigned char address);
unsigned int SetDDGExternalOutputEnabled(at_u32 uiIndex, at_u32 uiEnabled);
unsigned int SetDDGExternalOutputPolarity(at_u32 uiIndex, at_u32 uiPolarity);
unsigned int SetDDGExternalOutputTime(at_u32 uiIndex, at_u64 uiDelay, at_u64 uiWidth);
unsigned int SetDDGGain(int gain);
unsigned int SetDDGGateStep(double step_Renamed);
unsigned int SetDDGGateTime(at_u64 uiDelay, at_u64 uiWidth);
unsigned int SetDDGInsertionDelay(int state);
unsigned int SetDDGIntelligate(int state);
unsigned int SetDDGIOC(int state);
unsigned int SetDDGIOCFrequency(double frequency);
unsigned int SetDDGIOCNumber(unsigned long numberPulses);
unsigned int SetDDGIOCPeriod(at_u64 period);

// DDG Lite functions
unsigned int SetDDGLiteGlobalControlByte(unsigned char control);
unsigned int SetDDGLiteControlByte(AT_DDGLiteChannelId channel, unsigned char control);
unsigned int SetDDGLiteInitialDelay(AT_DDGLiteChannelId channel, float fDelay);
unsigned int SetDDGLitePulseWidth(AT_DDGLiteChannelId channel, float fWidth);
unsigned int SetDDGLiteInterPulseDelay(AT_DDGLiteChannelId channel, float fDelay);
unsigned int SetDDGLitePulsesPerExposure(AT_DDGLiteChannelId channel, at_u32 ui32Pulses);

unsigned int SetDDGStepCoefficients(at_u32 mode, double p1, double p2);
unsigned int SetDDGStepMode(at_u32 mode);
unsigned int SetDDGTimes(double t0, double t1, double t2);
unsigned int SetDDGTriggerMode(int mode);
unsigned int SetDDGVariableGateStep(int mode, double p1, double p2);
unsigned int SetDelayGenerator(int board, short address, int typ);
unsigned int SetDMAParameters(int MaxImagesPerDMA, float SecondsPerDMA);
// unsigned int SetDriverEvent(HANDLE driverEvent);
unsigned int SetEMAdvanced(int state);
unsigned int SetEMCCDGain(int gain);
unsigned int SetEMClockCompensation(int EMClockCompensationFlag);
unsigned int SetEMGainMode(int mode);
unsigned int SetExposureTime(float time);
unsigned int SetExternalTriggerTermination(at_u32 uiTermination);
unsigned int SetFanMode(int mode);
unsigned int SetFastExtTrigger(int mode);
unsigned int SetFastKinetics(int exposedRows, int seriesLength, float time, int mode, int hbin, int vbin);
unsigned int SetFastKineticsEx(int exposedRows, int seriesLength, float time, int mode, int hbin, int vbin, int offset);
unsigned int SetFilterMode(int mode);
unsigned int SetFilterParameters(int width, float sensitivity, int range, float accept, int smooth, int noise);
unsigned int SetFKVShiftSpeed(int index);
unsigned int SetFPDP(int state);
unsigned int SetFrameTransferMode(int mode);
unsigned int SetFullImage(int hbin, int vbin);
unsigned int SetFVBHBin(int bin);
unsigned int SetGain(int gain);
unsigned int SetGate(float delay, float width, float stepRenamed);
unsigned int SetGateMode(int gatemode);
unsigned int SetHighCapacity(int state);
unsigned int SetHorizontalSpeed(int index);
unsigned int SetHSSpeed(int typ, int index);
unsigned int SetImage(int hbin, int vbin, int hstart, int hend, int vstart, int vend);
unsigned int SetImageFlip(int iHFlip, int iVFlip);
unsigned int SetImageRotate(int iRotate);
unsigned int SetIsolatedCropMode(int active, int cropheight, int cropwidth, int vbin, int hbin);
unsigned int SetKineticCycleTime(float time);
unsigned int SetMCPGain(int gain);
unsigned int SetMCPGating(int gating);
unsigned int SetMessageWindow(int wnd);
unsigned int SetMetaData(int state);
unsigned int SetMultiTrack(int number, int height, int offset, int * bottom, int * gap);
unsigned int SetMultiTrackHBin(int bin);
unsigned int SetMultiTrackHRange(int iStart, int iEnd);
unsigned int SetMultiTrackScan(int trackHeight, int numberTracks, int iSIHStart, int iSIHEnd, int trackHBinning, int trackVBinning, int trackGap, int trackOffset, int trackSkip, int numberSubFrames);
unsigned int SetNextAddress(at_32 * data, at_32 lowAdd, at_32 highAdd, at_32 length, at_32 physical);
unsigned int SetNextAddress16(at_32 * data, at_32 lowAdd, at_32 highAdd, at_32 length, at_32 physical);
unsigned int SetNumberAccumulations(int number);
unsigned int SetNumberKinetics(int number);
unsigned int SetNumberPrescans(int iNumber);
unsigned int SetOutputAmplifier(int typ);
unsigned int SetOverlapMode(int mode);
unsigned int SetPCIMode(int mode, int value);
unsigned int SetPhotonCounting(int state);
unsigned int SetPhotonCountingThreshold(at_32 min, at_32 max);
unsigned int SetPixelMode(int bitdepth, int colormode);
unsigned int SetPreAmpGain(int index);
unsigned int SetRandomTracks(int numTracks, int * areas);
unsigned int SetReadMode(int mode);
unsigned int SetRegisterDump(int mode);
unsigned int SetRingExposureTimes(int numTimes, float * times);
// unsigned int SetSaturationEvent(HANDLE saturationEvent);
unsigned int SetShutter(int typ, int mode, int closingtime, int openingtime);
unsigned int SetShutterEx(int typ, int mode, int closingtime, int openingtime, int extmode);
unsigned int SetShutters(int typ, int mode, int closingtime, int openingtime, int exttype, int extmode, int dummy1, int dummy2);
unsigned int SetSifComment(char * comment);
unsigned int SetSingleTrack(int centre, int height);
unsigned int SetSingleTrackHBin(int bin);
unsigned int SetSpool(int active, int method, char * path, int framebuffersize);
unsigned int SetStorageMode(at_32 mode);
unsigned int SetTemperature(int temperature);
// unsigned int SetTemperatureEvent(HANDLE temperatureEvent);
unsigned int SetTriggerMode(int mode);
unsigned int SetTriggerInvert(int mode);
unsigned int SetIODirection(int index, int iDirection);
unsigned int SetIOLevel(int index, int iLevel);
// unsigned int SetUserEvent(HANDLE userEvent);
unsigned int SetUSGenomics(at_32 width, at_32 height);
unsigned int SetVerticalRowBuffer(int rows);
unsigned int SetVerticalSpeed(int index);
unsigned int SetVirtualChip(int state);
unsigned int SetVSAmplitude(int index);
unsigned int SetVSSpeed(int index);
unsigned int ShutDown();
unsigned int StartAcquisition();
unsigned int UnMapPhysicalAddress();
unsigned int WaitForAcquisition();
unsigned int WaitForAcquisitionByHandle(at_32 cameraHandle);
unsigned int WaitForAcquisitionByHandleTimeOut(long cameraHandle, int iTimeOutMs);
unsigned int WaitForAcquisitionTimeOut(int iTimeOutMs);
unsigned int WhiteBalance(unsigned short * wRed, unsigned short * wGreen, unsigned short * wBlue, float * fRelR, float * fRelB, WhiteBalanceInfo * info);

#define DRV_ERROR_CODES 20001
#define DRV_SUCCESS 20002
#define DRV_VXDNOTINSTALLED 20003
#define DRV_ERROR_SCAN 20004
#define DRV_ERROR_CHECK_SUM 20005
#define DRV_ERROR_FILELOAD 20006
#define DRV_UNKNOWN_FUNCTION 20007
#define DRV_ERROR_VXD_INIT 20008
#define DRV_ERROR_ADDRESS 20009
#define DRV_ERROR_PAGELOCK 20010
#define DRV_ERROR_PAGEUNLOCK 20011
#define DRV_ERROR_BOARDTEST 20012
#define DRV_ERROR_ACK 20013
#define DRV_ERROR_UP_FIFO 20014
#define DRV_ERROR_PATTERN 20015

#define DRV_ACQUISITION_ERRORS 20017
#define DRV_ACQ_BUFFER 20018
#define DRV_ACQ_DOWNFIFO_FULL 20019
#define DRV_PROC_UNKONWN_INSTRUCTION 20020
#define DRV_ILLEGAL_OP_CODE 20021
#define DRV_KINETIC_TIME_NOT_MET 20022
#define DRV_ACCUM_TIME_NOT_MET 20023
#define DRV_NO_NEW_DATA 20024
#define KERN_MEM_ERROR 20025
#define DRV_SPOOLERROR 20026
#define DRV_SPOOLSETUPERROR 20027
#define DRV_FILESIZELIMITERROR 20028
#define DRV_ERROR_FILESAVE 20029

#define DRV_TEMPERATURE_CODES 20033
#define DRV_TEMPERATURE_OFF 20034
#define DRV_TEMPERATURE_NOT_STABILIZED 20035
#define DRV_TEMPERATURE_STABILIZED 20036
#define DRV_TEMPERATURE_NOT_REACHED 20037
#define DRV_TEMPERATURE_OUT_RANGE 20038
#define DRV_TEMPERATURE_NOT_SUPPORTED 20039
#define DRV_TEMPERATURE_DRIFT 20040

#define DRV_TEMP_CODES 20033
#define DRV_TEMP_OFF 20034
#define DRV_TEMP_NOT_STABILIZED 20035
#define DRV_TEMP_STABILIZED 20036
#define DRV_TEMP_NOT_REACHED 20037
#define DRV_TEMP_OUT_RANGE 20038
#define DRV_TEMP_NOT_SUPPORTED 20039
#define DRV_TEMP_DRIFT 20040

#define DRV_GENERAL_ERRORS 20049
#define DRV_INVALID_AUX 20050
#define DRV_COF_NOTLOADED 20051
#define DRV_FPGAPROG 20052
#define DRV_FLEXERROR 20053
#define DRV_GPIBERROR 20054
#define DRV_EEPROMVERSIONERROR 20055

#define DRV_DATATYPE 20064
#define DRV_DRIVER_ERRORS 20065
#define DRV_P1INVALID 20066
#define DRV_P2INVALID 20067
#define DRV_P3INVALID 20068
#define DRV_P4INVALID 20069
#define DRV_INIERROR 20070
#define DRV_COFERROR 20071
#define DRV_ACQUIRING 20072
#define DRV_IDLE 20073
#define DRV_TEMPCYCLE 20074
#define DRV_NOT_INITIALIZED 20075
#define DRV_P5INVALID 20076
#define DRV_P6INVALID 20077
#define DRV_INVALID_MODE 20078
#define DRV_INVALID_FILTER 20079

#define DRV_I2CERRORS 20080
#define DRV_I2CDEVNOTFOUND 20081
#define DRV_I2CTIMEOUT 20082
#define DRV_P7INVALID 20083
#define DRV_P8INVALID 20084
#define DRV_P9INVALID 20085
#define DRV_P10INVALID 20086

#define DRV_USBERROR 20089
#define DRV_IOCERROR 20090
#define DRV_VRMVERSIONERROR 20091
#define DRV_USB_INTERRUPT_ENDPOINT_ERROR 20093
#define DRV_RANDOM_TRACK_ERROR 20094
#define DRV_INVALID_TRIGGER_MODE 20095
#define DRV_LOAD_FIRMWARE_ERROR 20096
#define DRV_DIVIDE_BY_ZERO_ERROR 20097
#define DRV_INVALID_RINGEXPOSURES 20098
#define DRV_BINNING_ERROR 20099
#define DRV_INVALID_AMPLIFIER 20100

#define DRV_ERROR_NOCAMERA 20990
#define DRV_NOT_SUPPORTED 20991
#define DRV_NOT_AVAILABLE 20992

#define DRV_ERROR_MAP 20115
#define DRV_ERROR_UNMAP 20116
#define DRV_ERROR_MDL 20117
#define DRV_ERROR_UNMDL 20118
#define DRV_ERROR_BUFFSIZE 20119
#define DRV_ERROR_NOHANDLE 20121

#define DRV_GATING_NOT_AVAILABLE 20130
#define DRV_FPGA_VOLTAGE_ERROR 20131

#define DRV_OW_CMD_FAIL 20150
#define DRV_OWMEMORY_BAD_ADDR 20151
#define DRV_OWCMD_NOT_AVAILABLE 20152
#define DRV_OW_NO_SLAVES 20153
#define DRV_OW_NOT_INITIALIZED 20154
#define DRV_OW_ERROR_SLAVE_NUM 20155
#define DRV_MSTIMINGS_ERROR 20156

#define AC_ACQMODE_SINGLE 1
#define AC_ACQMODE_VIDEO 2
#define AC_ACQMODE_ACCUMULATE 4
#define AC_ACQMODE_KINETIC 8
#define AC_ACQMODE_FRAMETRANSFER 16
#define AC_ACQMODE_FASTKINETICS 32
#define AC_ACQMODE_OVERLAP 64

#define AC_READMODE_FULLIMAGE 1
#define AC_READMODE_SUBIMAGE 2
#define AC_READMODE_SINGLETRACK 4
#define AC_READMODE_FVB 8
#define AC_READMODE_MULTITRACK 16
#define AC_READMODE_RANDOMTRACK 32
#define AC_READMODE_MULTITRACKSCAN 64

#define AC_TRIGGERMODE_INTERNAL 1
#define AC_TRIGGERMODE_EXTERNAL 2
#define AC_TRIGGERMODE_EXTERNAL_FVB_EM 4
#define AC_TRIGGERMODE_CONTINUOUS 8
#define AC_TRIGGERMODE_EXTERNALSTART 16
#define AC_TRIGGERMODE_EXTERNALEXPOSURE 32
#define AC_TRIGGERMODE_INVERTED 0x40

// Deprecated for AC_TRIGGERMODE_EXTERNALEXPOSURE
#define AC_TRIGGERMODE_BULB 32

#define AC_CAMERATYPE_PDA 0
#define AC_CAMERATYPE_IXON 1
#define AC_CAMERATYPE_ICCD 2
#define AC_CAMERATYPE_EMCCD 3
#define AC_CAMERATYPE_CCD 4
#define AC_CAMERATYPE_ISTAR 5
#define AC_CAMERATYPE_VIDEO 6
#define AC_CAMERATYPE_IDUS 7
#define AC_CAMERATYPE_NEWTON 8
#define AC_CAMERATYPE_SURCAM 9
#define AC_CAMERATYPE_USBICCD 10
#define AC_CAMERATYPE_LUCA 11
#define AC_CAMERATYPE_RESERVED 12
#define AC_CAMERATYPE_IKON 13
#define AC_CAMERATYPE_INGAAS 14
#define AC_CAMERATYPE_IVAC 15
#define AC_CAMERATYPE_UNPROGRAMMED 16
#define AC_CAMERATYPE_CLARA 17
#define AC_CAMERATYPE_USBISTAR 18

#define AC_PIXELMODE_8BIT 1
#define AC_PIXELMODE_14BIT 2
#define AC_PIXELMODE_16BIT 4
#define AC_PIXELMODE_32BIT 8

#define AC_PIXELMODE_MONO 0x000000
#define AC_PIXELMODE_RGB 0x010000
#define AC_PIXELMODE_CMY 0x020000

#define AC_SETFUNCTION_VREADOUT 0x01
#define AC_SETFUNCTION_HREADOUT 0x02
#define AC_SETFUNCTION_TEMPERATURE 0x04
#define AC_SETFUNCTION_MCPGAIN 0x08
#define AC_SETFUNCTION_EMCCDGAIN 0x10
#define AC_SETFUNCTION_BASELINECLAMP 0x20
#define AC_SETFUNCTION_VSAMPLITUDE 0x40
#define AC_SETFUNCTION_HIGHCAPACITY 0x80
#define AC_SETFUNCTION_BASELINEOFFSET 0x0100
#define AC_SETFUNCTION_PREAMPGAIN 0x0200
#define AC_SETFUNCTION_CROPMODE 0x0400
#define AC_SETFUNCTION_DMAPARAMETERS 0x0800
#define AC_SETFUNCTION_HORIZONTALBIN 0x1000
#define AC_SETFUNCTION_MULTITRACKHRANGE 0x2000
#define AC_SETFUNCTION_RANDOMTRACKNOGAPS 0x4000
#define AC_SETFUNCTION_EMADVANCED 0x8000
#define AC_SETFUNCTION_GATEMODE 0x010000
#define AC_SETFUNCTION_DDGTIMES 0x020000
#define AC_SETFUNCTION_IOC 0x040000
#define AC_SETFUNCTION_INTELLIGATE 0x080000
#define AC_SETFUNCTION_INSERTION_DELAY 0x100000
#define AC_SETFUNCTION_GATESTEP 0x200000

// Deprecated for AC_SETFUNCTION_MCPGAIN
#define AC_SETFUNCTION_GAIN 8
#define AC_SETFUNCTION_ICCDGAIN 8

#define AC_GETFUNCTION_TEMPERATURE 0x01
#define AC_GETFUNCTION_TARGETTEMPERATURE 0x02
#define AC_GETFUNCTION_TEMPERATURERANGE 0x04
#define AC_GETFUNCTION_DETECTORSIZE 0x08
#define AC_GETFUNCTION_MCPGAIN 0x10
#define AC_GETFUNCTION_EMCCDGAIN 0x20
#define AC_GETFUNCTION_HVFLAG 0x40
#define AC_GETFUNCTION_GATEMODE 0x80
#define AC_GETFUNCTION_DDGTIMES 0x0100
#define AC_GETFUNCTION_IOC 0x0200
#define AC_GETFUNCTION_INTELLIGATE 0x0400
#define AC_GETFUNCTION_INSERTION_DELAY 0x0800
#define AC_GETFUNCTION_GATESTEP 0x1000
#define AC_GETFUNCTION_PHOSPHORSTATUS 0x2000
#define AC_GETFUNCTION_MCPGAINTABLE 0x4000

// Deprecated for AC_GETFUNCTION_MCPGAIN
#define AC_GETFUNCTION_GAIN 0x10
#define AC_GETFUNCTION_ICCDGAIN 0x10

#define AC_FEATURES_POLLING 1
#define AC_FEATURES_EVENTS 2
#define AC_FEATURES_SPOOLING 4
#define AC_FEATURES_SHUTTER 8
#define AC_FEATURES_SHUTTEREX 16
#define AC_FEATURES_EXTERNAL_I2C 32
#define AC_FEATURES_SATURATIONEVENT 64
#define AC_FEATURES_FANCONTROL 128
#define AC_FEATURES_MIDFANCONTROL 256
#define AC_FEATURES_TEMPERATUREDURINGACQUISITION 512
#define AC_FEATURES_KEEPCLEANCONTROL 1024
#define AC_FEATURES_DDGLITE 0x0800
#define AC_FEATURES_FTEXTERNALEXPOSURE 0x1000
#define AC_FEATURES_KINETICEXTERNALEXPOSURE 0x2000
#define AC_FEATURES_DACCONTROL 0x4000
#define AC_FEATURES_METADATA 0x8000
#define AC_FEATURES_IOCONTROL 0x10000

#define AC_EMGAIN_8BIT 1
#define AC_EMGAIN_12BIT 2
#define AC_EMGAIN_LINEAR12 4
#define AC_EMGAIN_REAL12 8

#ifdef __cplusplus
}
#endif

#endif

