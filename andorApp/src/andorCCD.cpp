/**
 * Area Detector driver for the Andor CCD.
 *
 * @author Matthew Pearson
 * @date June 2009
 *
 * Updated Dec 2011 for Asyn 4-17 and areaDetector 1-7 
 *
 * Major updates to get callbacks working, etc. by Mark Rivers Feb. 2011
 * Updated by Peter Heesterman to support multi-track operation Oct. 2019
 *
 */

#include <stdio.h>
#include <string.h>
#include <string>
#include <errno.h>

#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsEvent.h>
#include <epicsString.h>
#include <iocsh.h>
#include <epicsExit.h>

#include <libxml/parser.h>
#include <ADDriver.h>

#ifdef _WIN32
#include "ATMCD32D.h"
#else
#include "atmcdLXd.h"
#endif
#ifdef USE_SHAMROCK
  #include "ShamrockCIF.h"
#endif
#include "SPEHeader.h"

#include <epicsExport.h>
#include "andorCCD.h"

#define DRIVER_VERSION      2
#define DRIVER_REVISION     9
#define DRIVER_MODIFICATION 0

static const char *driverName = "andorCCD";

//Definitions of static class data members

const epicsInt32 AndorCCD::AImageFastKinetics = ADImageContinuous+1;

const epicsUInt32 AndorCCD::AASingle = 1;
const epicsUInt32 AndorCCD::AAAccumulate = 2;
const epicsUInt32 AndorCCD::AAKinetics = 3;
const epicsUInt32 AndorCCD::AAFastKinetics = 4;
const epicsUInt32 AndorCCD::AARunTillAbort = 5;
const epicsUInt32 AndorCCD::AATimeDelayedInt = 9;

const epicsUInt32 AndorCCD::ATInternal = 0;
const epicsUInt32 AndorCCD::ATExternal = 1;
const epicsUInt32 AndorCCD::ATExternalStart = 6;
const epicsUInt32 AndorCCD::ATExternalExposure = 7;
const epicsUInt32 AndorCCD::ATExternalFVB = 9;
const epicsUInt32 AndorCCD::ATSoftware = 10;

const epicsUInt32 AndorCCD::ASIdle = DRV_IDLE;
const epicsUInt32 AndorCCD::ASTempCycle = DRV_TEMPCYCLE;
const epicsUInt32 AndorCCD::ASAcquiring = DRV_ACQUIRING;
const epicsUInt32 AndorCCD::ASAccumTimeNotMet = DRV_ACCUM_TIME_NOT_MET;
const epicsUInt32 AndorCCD::ASKineticTimeNotMet = DRV_KINETIC_TIME_NOT_MET;
const epicsUInt32 AndorCCD::ASErrorAck = DRV_ERROR_ACK;
const epicsUInt32 AndorCCD::ASAcqBuffer = DRV_ACQ_BUFFER;
const epicsUInt32 AndorCCD::ASSpoolError = DRV_SPOOLERROR;

const epicsInt32 AndorCCD::ARFullVerticalBinning = 0;
const epicsInt32 AndorCCD::ARMultiTrack = 1;
const epicsInt32 AndorCCD::ARRandomTrack = 2;
const epicsInt32 AndorCCD::ARSingleTrack = 3;
const epicsInt32 AndorCCD::ARImage = 4;

const epicsInt32 AndorCCD::AShutterFullyAuto    = 0;
const epicsInt32 AndorCCD::AShutterAlwaysOpen   = 1;
const epicsInt32 AndorCCD::AShutterAlwaysClosed = 2;
const epicsInt32 AndorCCD::AShutterOpenFVP      = 4;
const epicsInt32 AndorCCD::AShutterOpenAny      = 5;

const epicsInt32 AndorCCD::AFFTIFF = 0;
const epicsInt32 AndorCCD::AFFBMP  = 1;
const epicsInt32 AndorCCD::AFFSIF  = 2;
const epicsInt32 AndorCCD::AFFEDF  = 3;
const epicsInt32 AndorCCD::AFFRAW  = 4;
const epicsInt32 AndorCCD::AFFFITS = 5;
const epicsInt32 AndorCCD::AFFSPE  = 6;

//C Function prototypes to tie in with EPICS
static void andorStatusTaskC(void *drvPvt);
static void andorDataTaskC(void *drvPvt);
static void exitHandler(void *drvPvt);

/** Constructor for Andor driver; most parameters are simply passed to ADDriver::ADDriver.
  * After calling the base class constructor this method creates a thread to collect the detector data, 
  * and sets reasonable default values the parameters defined in this class, asynNDArrayDriver, and ADDriver.
  * \param[in] portName The name of the asyn port driver to be created.
  * \param[in] installPath The path to the Andor directory containing the detector INI files, etc.
  *            This can be specified as an empty string ("") for new detectors that don't use the INI
  *            files on Windows, but must be a valid path on Linux.
  * \param[in] cameraSerial The serial number of the desired camera.
  * \param[in] shamrockID The index number of the Shamrock spectrograph, if installed.
  *            0 is the first Shamrock in the system.  Ignored if there are no Shamrocks.  
  * \param[in] maxBuffers The maximum number of NDArray buffers that the NDArrayPool for this driver is 
  *            allowed to allocate. Set this to -1 to allow an unlimited number of buffers.
  * \param[in] maxMemory The maximum amount of memory that the NDArrayPool for this driver is 
  *            allowed to allocate. Set this to -1 to allow an unlimited amount of memory.
  * \param[in] priority The thread priority for the asyn port driver thread if ASYN_CANBLOCK is set in asynFlags.
  * \param[in] stackSize The stack size for the asyn port driver thread if ASYN_CANBLOCK is set in asynFlags.
  */
AndorCCD::AndorCCD(const char *portName, const char *installPath, int cameraSerial, int shamrockID,
                   int maxBuffers, size_t maxMemory, int priority, int stackSize)

  : ADDriver(portName, 1, 0, maxBuffers, maxMemory, 
             asynEnumMask, asynEnumMask,
             ASYN_CANBLOCK, 1, priority, stackSize),
    mExiting(false), mExited(0), mShamrockId(shamrockID), mMultiTrack(this), mSPEDoc(0), mInitOK(false)
{

  int status = asynSuccess;
  int i;
  int binX=1, binY=1, minX=0, minY=0, sizeX, sizeY;
  char model[256];
  char SDKVersion[256];
  char tempString[256];
  int serialNumber;
  unsigned int firmwareVersion;
  unsigned int firmwareBuild;
  unsigned int uTemp;
  static const char *functionName = "AndorCCD";

  if (installPath == NULL)
    strcpy(mInstallPath, "");
  else 
    mInstallPath = epicsStrDup(installPath);

  /* Create an EPICS exit handler */
  epicsAtExit(exitHandler, this);

  createParam(AndorCoolerParamString,             asynParamInt32, &AndorCoolerParam);
  createParam(AndorTempStatusMessageString,       asynParamOctet, &AndorTempStatusMessage);
  createParam(AndorMessageString,                 asynParamOctet, &AndorMessage);
  createParam(AndorShutterModeString,             asynParamInt32, &AndorShutterMode);
  createParam(AndorShutterExTTLString,            asynParamInt32, &AndorShutterExTTL);
  createParam(AndorPalFileNameString,             asynParamOctet, &AndorPalFileName);
  createParam(AndorAccumulatePeriodString,        asynParamFloat64, &AndorAccumulatePeriod);
  createParam(AndorPreAmpGainString,              asynParamInt32, &AndorPreAmpGain);
  createParam(AndorEmGainString,                  asynParamInt32, &AndorEmGain);
  createParam(AndorEmGainModeString,              asynParamInt32, &AndorEmGainMode);
  createParam(AndorEmGainAdvancedString,          asynParamInt32, &AndorEmGainAdvanced);
  createParam(AndorAdcSpeedString,                asynParamInt32, &AndorAdcSpeed);
  createParam(AndorBaselineClampString,           asynParamInt32, &AndorBaselineClamp);
  createParam(AndorReadOutModeString,             asynParamInt32, &AndorReadOutMode);
  createParam(AndorFrameTransferModeString,       asynParamInt32, &AndorFrameTransferMode);
  createParam(AndorVerticalShiftPeriodString,     asynParamInt32, &AndorVerticalShiftPeriod);
  createParam(AndorVerticalShiftAmplitudeString,  asynParamInt32, &AndorVerticalShiftAmplitude);


  // Create the epicsEvent for signaling to the status task when parameters should have changed.
  // This will cause it to do a poll immediately, rather than wait for the poll time period.
  this->statusEvent = epicsEventMustCreate(epicsEventEmpty);
  if (!this->statusEvent) {
    printf("%s:%s epicsEventCreate failure for start event\n", driverName, functionName);
    return;
  }

  // Use this to signal the data acquisition task that acquisition has started.
  this->dataEvent = epicsEventMustCreate(epicsEventEmpty);
  if (!this->dataEvent) {
    printf("%s:%s epicsEventCreate failure for data event\n", driverName, functionName);
    return;
  }

  // Initialize ADC enums
  for (i=0; i<MAX_ADC_SPEEDS; i++) {
    mADCSpeeds[i].EnumValue = i;
    mADCSpeeds[i].EnumString = (char *)calloc(MAX_ENUM_STRING_SIZE, sizeof(char));
  }

  // Initialize Pre-Amp enums
  for (i=0; i<MAX_PREAMP_GAINS; i++) {
    mPreAmpGains[i].EnumValue = i;
    mPreAmpGains[i].EnumString = (char *)calloc(MAX_ENUM_STRING_SIZE, sizeof(char));
  }

  // Initialize Vertical Shift Period enums
  for (i=0; i<MAX_VS_PERIODS; i++) {
    mVSPeriods[i].EnumValue = i;
    mVSPeriods[i].EnumString = (char *)calloc(MAX_ENUM_STRING_SIZE, sizeof(char));
  }

  // Initialize camera
  try {
    at_32 numCameras;
    checkStatus(GetAvailableCameras(&numCameras));
    bool cameraFound = false;
    for (i=0; i<numCameras; i++) {
      at_32 cameraHandle = -1;
      checkStatus(GetCameraHandle(i, &cameraHandle));
      checkStatus(SetCurrentCamera(cameraHandle));
      printf("%s:%s: initializing camera with handle %d\n", driverName, functionName, cameraHandle);
      unsigned long error = Initialize(mInstallPath);
      if (error == DRV_NOT_AVAILABLE) {
        // Is this the right way to detect if camera is used/busy/claimed?
        printf("%s:%s: camera with handle %d not available (already claimed?)\n",
               driverName, functionName, cameraHandle);
      } else if (error == DRV_SUCCESS) {
        checkStatus(GetCameraSerialNumber(&serialNumber));
        if ((cameraSerial == serialNumber) ||
          ((cameraSerial == 0) && (serialNumber != 0))) {
          cameraFound = true;
          break;
        }
      } else {
        printf("%s:%s: initialization error for camera handle %d: %ld\n",
               driverName, functionName, cameraHandle, error);
      }
      ShutDown();
    }
    if (! cameraFound) {
      asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
        "%s::%s camera not detected!\n", driverName, functionName);
      return;
    }
    printf("%s:%s: found camera with serial %d\n", driverName, functionName, serialNumber);

    setStringParam(AndorMessage, "Camera successfully initialized.");
    checkStatus(GetDetector(&sizeX, &sizeY));
    checkStatus(GetHeadModel(model));
    checkStatus(GetCameraSerialNumber(&serialNumber));
    checkStatus(GetHardwareVersion(&uTemp, &uTemp, &uTemp, 
                                   &uTemp, &firmwareVersion, &firmwareBuild));
    checkStatus(GetVersionInfo(AT_SDKVersion, SDKVersion, sizeof(SDKVersion)));
    checkStatus(SetReadMode(ARImage));
    checkStatus(SetImage(binX, binY, minX+1, minX+sizeX, minY+1, minY+sizeY));
    checkStatus(GetShutterMinTimes(&mMinShutterCloseTime, &mMinShutterOpenTime));
    try {
      checkStatus(GetFastestRecommendedVSSpeed(&mVSIndex, &mVSPeriod));
    } catch (const std::string &e) {
      asynPrint(pasynUserSelf, ASYN_TRACE_WARNING,
        "%s:%s: GetFastestRecommendedVSSpeed is not available: %lu: %s\n",
         driverName, functionName, e.c_str());
      mVSIndex = -1;
      mVSPeriod = 0.0f;
    }
    mCapabilities.ulSize = sizeof(mCapabilities);
    checkStatus(GetCapabilities(&mCapabilities));

    /* Get current temperature */
    float temperature;
    int coolerStatus;
    int maxTemp = 0;
    int minTemp = 0;
    checkStatus(IsCoolerOn(&coolerStatus));
    if (coolerStatus){
      checkStatus(GetTemperatureF(&temperature));
      checkStatus(GetTemperatureRange(&minTemp, &maxTemp));
      printf("%s:%s: current temperature is %f\n", driverName, functionName, temperature);
      if (static_cast<int>(temperature) < minTemp) {
        setDoubleParam(ADTemperature, minTemp);
      } else if (static_cast<int>(temperature) > maxTemp) {
        setDoubleParam(ADTemperature, maxTemp);
      } else {
        setDoubleParam(ADTemperature, temperature);
      }
    }
    callParamCallbacks();
  } catch (const std::string &e) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
      "%s:%s: %s\n",
      driverName, functionName, e.c_str());
    return;
  }
  

  /* Set some default values for parameters */
  status =  setStringParam(ADManufacturer, "Andor");
  status |= setStringParam(ADModel, model);
  epicsSnprintf(tempString, sizeof(tempString), "%u", serialNumber);
  status |= setStringParam(ADSerialNumber, tempString);
  epicsSnprintf(tempString, sizeof(tempString), "%d.%d", firmwareVersion, firmwareBuild);
  status |= setStringParam(ADFirmwareVersion, tempString);
  status |= setStringParam(ADSDKVersion, SDKVersion);
  epicsSnprintf(tempString, sizeof(tempString), "%d.%d.%d", 
                DRIVER_VERSION, DRIVER_REVISION, DRIVER_MODIFICATION);
  setStringParam(NDDriverVersion,tempString);
  status |= setIntegerParam(ADSizeX, sizeX);
  status |= setIntegerParam(ADSizeY, sizeY);
  status |= setIntegerParam(ADBinX, 1);
  status |= setIntegerParam(ADBinY, 1);
  status |= setIntegerParam(ADMinX, 0);
  status |= setIntegerParam(ADMinY, 0);
  status |= setIntegerParam(ADMaxSizeX, sizeX);
  status |= setIntegerParam(ADMaxSizeY, sizeY);  
  status |= setIntegerParam(ADImageMode, ADImageSingle);
  status |= setIntegerParam(ADTriggerMode, AndorCCD::ATInternal);
  mAcquireTime = 1.0;
  status |= setDoubleParam (ADAcquireTime, mAcquireTime);
  mAcquirePeriod = 5.0;
  status |= setDoubleParam (ADAcquirePeriod, mAcquirePeriod);
  status |= setIntegerParam(ADNumImages, 1);
  status |= setIntegerParam(ADNumExposures, 1);
  status |= setIntegerParam(NDArraySizeX, sizeX);
  status |= setIntegerParam(NDArraySizeY, sizeY);
  status |= setIntegerParam(NDDataType, NDUInt16);
  status |= setIntegerParam(NDArraySize, sizeX*sizeY*sizeof(epicsUInt16)); 
  mAccumulatePeriod = 2.0;
  status |= setDoubleParam(AndorAccumulatePeriod, mAccumulatePeriod); 
  status |= setIntegerParam(AndorEmGain, 0); 
  status |= setIntegerParam(AndorEmGainMode, 0); 
  status |= setIntegerParam(AndorEmGainAdvanced, 0); 
  status |= setIntegerParam(AndorAdcSpeed, 0);
  status |= setIntegerParam(AndorShutterExTTL, 1);
  status |= setIntegerParam(AndorShutterMode, AShutterFullyAuto);
  status |= setDoubleParam(ADShutterOpenDelay, 0.);
  status |= setDoubleParam(ADShutterCloseDelay, 0.);
  status |= setIntegerParam(AndorReadOutMode, ARImage);
  status |= setIntegerParam(AndorFrameTransferMode, 0);

  setupADCSpeeds();
  setupPreAmpGains();
  setupVerticalShiftPeriods();
  status |= setIntegerParam(AndorVerticalShiftPeriod, mVSIndex);
  status |= setIntegerParam(AndorVerticalShiftAmplitude, 0);
  status |= setupShutter(-1);

  callParamCallbacks();

  /* Send a signal to the poller task which will make it do a poll, and switch to the fast poll rate */
  epicsEventSignal(statusEvent);

  if (status) {
    printf("%s:%s: unable to set camera parameters\n", driverName, functionName);
    return;
  }

  // Define the polling periods for the status thread.
  mPollingPeriod = 0.2; // seconds
  mFastPollingPeriod = 0.05; // seconds

  mAcquiringData = 0;
  
  mSPEHeader = (tagCSMAHEAD *) calloc(1, sizeof(tagCSMAHEAD));
  
  if (stackSize == 0) stackSize = epicsThreadGetStackSize(epicsThreadStackMedium);

  /* Create the thread that updates the detector status */
  status = (epicsThreadCreate("AndorStatusTask",
                              epicsThreadPriorityMedium,
                              stackSize,
                              (EPICSTHREADFUNC)andorStatusTaskC,
                              this) == NULL);
  if (status) {
    printf("%s:%s: epicsThreadCreate failure for status task\n",
           driverName, functionName);
    return;
  }

  
  /* Create the thread that does data readout */
  status = (epicsThreadCreate("AndorDataTask",
                              epicsThreadPriorityMedium,
                              stackSize,
                              (EPICSTHREADFUNC)andorDataTaskC,
                              this) == NULL);
  if (status) {
    printf("%s:%s: epicsThreadCreate failure for data task\n",
           driverName, functionName);
    return;
  }
  printf("CCD initialized OK!\n");
  mInitOK = true;
}

/**
 * Destructor.  Free resources and closes the Andor library
 */
AndorCCD::~AndorCCD() 
{
  static const char *functionName = "~AndorCCD";
  asynStatus status;

  mExiting = true;
  this->lock();
  printf("%s::%s Shutdown and freeing up memory...\n", driverName, functionName);
  try {
    int acquireStatus;
    checkStatus(GetStatus(&acquireStatus));
    if (acquireStatus == DRV_ACQUIRING)
      checkStatus(AbortAcquisition());
    epicsEventSignal(dataEvent);
    checkStatus(FreeInternalMemory());
    checkStatus(ShutDown());
  } catch (const std::string &e) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
      "%s:%s: %s\n",
      driverName, functionName, e.c_str());
      status = asynError;
  }
  this->unlock();
  while ((mExited < 2) && (status != asynError))
      epicsThreadSleep(0.2);
}


/**
 * Exit handler, delete the AndorCCD object.
 */

static void exitHandler(void *drvPvt)
{
  AndorCCD *pAndorCCD = (AndorCCD *)drvPvt;
  delete pAndorCCD;
}


void AndorCCD::setupPreAmpGains()
{
  int i;
  AndorADCSpeed_t *pSpeed;
  AndorPreAmpGain_t *pGain = mPreAmpGains;
  int isAvailable;
  int adcSpeed;
  float gain;
  char *enumStrings[MAX_PREAMP_GAINS];
  int enumValues[MAX_PREAMP_GAINS];
  int enumSeverities[MAX_PREAMP_GAINS];
  static const char *functionName = "setupPreAmpGains";
  
  mNumPreAmpGains = 0;
  getIntegerParam(AndorAdcSpeed, &adcSpeed);
  pSpeed = &mADCSpeeds[adcSpeed];

  try{
    for (i=0; i<mTotalPreAmpGains; i++) {
      checkStatus(IsPreAmpGainAvailable(pSpeed->ADCIndex, pSpeed->AmpIndex, pSpeed->HSSpeedIndex, 
                  i, &isAvailable));
      if (isAvailable) {
        checkStatus(GetPreAmpGain(i, &gain));
        epicsSnprintf(pGain->EnumString, MAX_ENUM_STRING_SIZE, "%.2f", gain);
        pGain->EnumValue = i;
        pGain->Gain = gain;
        mNumPreAmpGains++;
        if (mNumPreAmpGains >= MAX_PREAMP_GAINS) break;
        pGain++;
      }
    }
    for (i=0; i<mNumPreAmpGains; i++) {
      enumStrings[i] = mPreAmpGains[i].EnumString;
      enumValues[i] = mPreAmpGains[i].EnumValue;
      enumSeverities[i] = 0;
    }
    doCallbacksEnum(enumStrings, enumValues, enumSeverities, 
                    mNumPreAmpGains, AndorPreAmpGain, 0);
  } catch (const std::string &e) {
      asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: %s\n", driverName, functionName, e.c_str());
  }
}

asynStatus AndorCCD::readEnum(asynUser *pasynUser, char *strings[], int values[], int severities[], 
                              size_t nElements, size_t *nIn)
{
  int function = pasynUser->reason;
  int i;

  if (function == AndorAdcSpeed) {
    for (i=0; ((i<mNumADCSpeeds) && (i<(int)nElements)); i++) {
      if (strings[i]) free(strings[i]);
      strings[i] = epicsStrDup(mADCSpeeds[i].EnumString);
      values[i] = mADCSpeeds[i].EnumValue;
      severities[i] = 0;
    }
  }
  else if (function == AndorPreAmpGain) {
    for (i=0; ((i<mNumPreAmpGains) && (i<(int)nElements)); i++) {
      if (strings[i]) free(strings[i]);
      strings[i] = epicsStrDup(mPreAmpGains[i].EnumString);
      values[i] = mPreAmpGains[i].EnumValue;
      severities[i] = 0;
    }
  }
  else if (function == AndorVerticalShiftPeriod) {
    for (i=0; ((i<mNumVSPeriods) && (i<(int)nElements)); i++) {
      if (strings[i]) free(strings[i]);
      strings[i] = epicsStrDup(mVSPeriods[i].EnumString);
      values[i] = mVSPeriods[i].EnumValue;
      severities[i] = 0;
    }
  }
  else {
    *nIn = 0;
    return asynError;
  }
  *nIn = i;
  return asynSuccess;   
}

void AndorCCD::setupADCSpeeds()
{
  int i, j, k, numHSSpeeds, bitDepth;
  float HSSpeed;
  AndorADCSpeed_t *pSpeed = mADCSpeeds;

  mNumADCSpeeds = 0;
  checkStatus(GetNumberAmp(&mNumAmps));
  checkStatus(GetNumberADChannels(&mNumADCs));
  checkStatus(GetNumberPreAmpGains(&mTotalPreAmpGains));
  for (i=0; i<mNumADCs; i++) {
    checkStatus(GetBitDepth(i, &bitDepth));
    for (j=0; j<mNumAmps; j++) {
      checkStatus(GetNumberHSSpeeds(i, j, &numHSSpeeds));
      for (k=0; k<numHSSpeeds; k++ ) {
        checkStatus(GetHSSpeed(i, j, k, &HSSpeed));
        pSpeed->ADCIndex = i;
        pSpeed->AmpIndex = j;
        pSpeed->HSSpeedIndex = k;
        pSpeed->BitDepth = bitDepth;
        pSpeed->HSSpeed = HSSpeed;
        epicsSnprintf(pSpeed->EnumString, MAX_ENUM_STRING_SIZE, 
                      "%.2f MHz", HSSpeed);
        mNumADCSpeeds++;
        if (mNumADCSpeeds >= MAX_ADC_SPEEDS) return;
        pSpeed++;
      }
    }
  }
}

void AndorCCD::setupVerticalShiftPeriods()
{
    int i, numVSPeriods;
    float VSPeriod;
    AndorVSPeriod_t *pPeriod = mVSPeriods;
    static const char *functionName = "setupVerticalShiftPeriods";

    mNumVSPeriods = 0;
    unsigned int error = GetNumberVSSpeeds(&mNumVSPeriods);
    if (error == DRV_SUCCESS && mNumVSPeriods > 0) {
        for (i=0; i<numVSPeriods; i++) {
            checkStatus(GetVSSpeed(i, &VSPeriod));
            pPeriod->Index = i;
            pPeriod->Period = VSPeriod;
            epicsSnprintf(pPeriod->EnumString, MAX_ENUM_STRING_SIZE, 
                          "%.2f us", VSPeriod);
            mNumVSPeriods++;
            if (mNumVSPeriods >= MAX_VS_PERIODS) return;
            pPeriod++;
        }
    } else {
        printf("%s:%s: unable to set camera parameters\n", driverName, functionName);
    }
}


/** Report status of the driver.
  * Prints details about the detector in us if details>0.
  * It then calls the ADDriver::report() method.
  * \param[in] fp File pointed passed by caller where the output is written to.
  * \param[in] details Controls the level of detail in the report. */
void AndorCCD::report(FILE *fp, int details)
{
  int param1;
  float fParam1;
  int xsize, ysize;
  int i;
  char sParam[256];
  unsigned int uIntParam1;
  unsigned int uIntParam2;
  unsigned int uIntParam3;
  unsigned int uIntParam4;
  unsigned int uIntParam5;
  unsigned int uIntParam6;
  AndorADCSpeed_t *pSpeed;
  int vsIndex;
  float vsPeriod;
  static const char *functionName = "report";

  fprintf(fp, "Andor CCD port=%s\n", this->portName);
  if (details > 0) {
    try {
      checkStatus(GetHeadModel(sParam));
      fprintf(fp, "  Model: %s\n", sParam);
      checkStatus(GetCameraSerialNumber(&param1));
      fprintf(fp, "  Serial number: %d\n", param1); 
      checkStatus(GetHardwareVersion(&uIntParam1, &uIntParam2, &uIntParam3, 
                                     &uIntParam4, &uIntParam5, &uIntParam6));
      fprintf(fp, "  PCB version: %d\n", uIntParam1);
      fprintf(fp, "  Flex file version: %d\n", uIntParam2);
      fprintf(fp, "  Firmware version: %d\n", uIntParam5);
      fprintf(fp, "  Firmware build: %d\n", uIntParam6);
      checkStatus(GetVersionInfo(AT_SDKVersion, sParam, sizeof(sParam)));
      fprintf(fp, "  SDK version: %s\n", sParam);
      checkStatus(GetVersionInfo(AT_DeviceDriverVersion, sParam, sizeof(sParam)));
      fprintf(fp, "  Device driver version: %s\n", sParam);
      getIntegerParam(ADMaxSizeX, &xsize);
      getIntegerParam(ADMaxSizeY, &ysize);
      fprintf(fp, "  X pixels: %d\n", xsize);
      fprintf(fp, "  Y pixels: %d\n", ysize);
      fprintf(fp, "  Number of amplifier channels: %d\n", mNumAmps);
      fprintf(fp, "  Number of ADC channels: %d\n", mNumADCs);
      fprintf(fp, "  Number of pre-amp gains (total): %d\n", mTotalPreAmpGains);
      for (i=0; i<mTotalPreAmpGains; i++) {
        checkStatus(GetPreAmpGain(i, &fParam1));
        fprintf(fp, "    Gain[%d]: %f\n", i, fParam1);
      }
      fprintf(fp, "  Total ADC speeds: %d\n", mNumADCSpeeds);
      for (i=0; i<mNumADCSpeeds; i++) {
        pSpeed = &mADCSpeeds[i];
        fprintf(fp, "    Amp=%d, ADC=%d, bitDepth=%d, HSSpeedIndex=%d, HSSpeed=%f\n",
                pSpeed->AmpIndex, pSpeed->ADCIndex, pSpeed->BitDepth, pSpeed->HSSpeedIndex, pSpeed->HSSpeed);
      }
      fprintf(fp, "  Pre-amp gains available: %d\n", mNumPreAmpGains);
      for (i=0; i<mNumPreAmpGains; i++) {
        fprintf(fp, "    Index=%d, Gain=%f\n",
                mPreAmpGains[i].EnumValue, mPreAmpGains[i].Gain);
      }
      
      if (mNumVSPeriods == 0) {
        fprintf(fp, "  No Vertical Shift Periods available\n");
      } else {
        fprintf(fp, "  Vertical Shift Periods available: %d\n", mNumVSPeriods);
        for (i=0; i<mNumVSPeriods; i++) {
          fprintf(fp, "    Index=%d, Period=%f [us per pixel shift]\n",
                  mVSPeriods[i].EnumValue, mVSPeriods[i].Period);
        }
        fprintf(fp, "  Fastest recommended Vertical Shift Period:\n");
        checkStatus(GetFastestRecommendedVSSpeed(&vsIndex, &vsPeriod));
        fprintf(fp, "    Index=%d, Period=%f [us per pixel shift]\n", vsIndex, vsPeriod);
      }

      fprintf(fp, "  Capabilities\n");
      fprintf(fp, "        AcqModes=0x%X\n", (int)mCapabilities.ulAcqModes);
      fprintf(fp, "       ReadModes=0x%X\n", (int)mCapabilities.ulReadModes);
      fprintf(fp, "     FTReadModes=0x%X\n", (int)mCapabilities.ulFTReadModes);
      fprintf(fp, "    TriggerModes=0x%X\n", (int)mCapabilities.ulTriggerModes);
      fprintf(fp, "      CameraType=%d\n",   (int)mCapabilities.ulCameraType);
      fprintf(fp, "      PixelModes=0x%X\n", (int)mCapabilities.ulPixelMode);
      fprintf(fp, "    SetFunctions=0x%X\n", (int)mCapabilities.ulSetFunctions);
      fprintf(fp, "    GetFunctions=0x%X\n", (int)mCapabilities.ulGetFunctions);
      fprintf(fp, "        Features=0x%X\n", (int)mCapabilities.ulFeatures);
      fprintf(fp, "         PCI MHz=%d\n",   (int)mCapabilities.ulPCICard);
      fprintf(fp, "          EMGain=0x%X\n", (int)mCapabilities.ulEMGainCapability);

    } catch (const std::string &e) {
      asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
        "%s:%s: %s\n",
        driverName, functionName, e.c_str());
    }
  }
  // Call the base class method
  ADDriver::report(fp, details);
}


/** Called when asyn clients call pasynInt32->write().
  * This function performs actions for some parameters, including ADAcquire, ADBinX, etc.
  * For all parameters it sets the value in the parameter library and calls any registered callbacks..
  * \param[in] pasynUser pasynUser structure that encodes the reason and address.
  * \param[in] value Value to write. */
asynStatus AndorCCD::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    int function = pasynUser->reason;
    int adstatus = 0;
    epicsInt32 oldValue;

    asynStatus status = asynSuccess;
    static const char *functionName = "writeInt32";

    // Set in param lib so the user sees a readback straight away. Save a backup in case of errors.
    getIntegerParam(function, &oldValue);
    status = setIntegerParam(function, value);

    if (function == ADAcquire) {
      getIntegerParam(ADStatus, &adstatus);
      if (value && (adstatus == ADStatusIdle)) {
        // Start the acqusition here, then send an event to the dataTask at the end of this function
        try {
          // Set up acquisition
          mAcquiringData = 1;
          status = setupAcquisition();
          if (status != asynSuccess) throw std::string("Setup acquisition failed");
          // Open the shutter if we control it
          int adShutterMode;
          getIntegerParam(ADShutterMode, &adShutterMode);
          if (adShutterMode == ADShutterModeEPICS) {
            ADDriver::setShutter(ADShutterOpen);
          }
          // Start acquisition
          asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
            "%s:%s:, StartAcquisition()\n", 
            driverName, functionName);
          checkStatus(StartAcquisition());
          // Reset the counters
          setIntegerParam(ADNumImagesCounter, 0);
          setIntegerParam(ADNumExposuresCounter, 0);
        } catch (const std::string &e) {
          asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: %s\n",
            driverName, functionName, e.c_str());
          status = asynError;
        }
      }
      if (!value && (adstatus != ADStatusIdle)) {
        try {
          asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
            "%s:%s:, AbortAcquisition()\n", 
            driverName, functionName);
          checkStatus(AbortAcquisition());
          mAcquiringData = 0;
          asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
            "%s:%s:, FreeInternalMemory()\n", 
            driverName, functionName);
          checkStatus(FreeInternalMemory());
          asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
            "%s:%s:, CancelWait()\n", 
            driverName, functionName);
          checkStatus(CancelWait());
        } catch (const std::string &e) {
          asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: %s\n",
            driverName, functionName, e.c_str());
          status = asynError;
        } 
      }
    }
    else if ((function == ADNumExposures)   || (function == ADNumImages)            ||
             (function == ADImageMode)                                              ||
             (function == ADBinX)           || (function == ADBinY)                 ||
             (function == ADMinX)           || (function == ADMinY)                 ||
             (function == ADSizeX)          || (function == ADSizeY)                ||
             (function == ADReverseX)       || (function == ADReverseY)             ||
             (function == ADTriggerMode)    || (function == AndorEmGain)            || 
             (function == AndorEmGainMode)  || (function == AndorEmGainAdvanced)    ||
             (function == AndorAdcSpeed)    || (function == AndorPreAmpGain)        ||
             (function == AndorReadOutMode) || (function == AndorFrameTransferMode) ||
             (function == AndorVerticalShiftPeriod) || (function == AndorVerticalShiftAmplitude)) {
      status = setupAcquisition();
      if (function == AndorAdcSpeed) setupPreAmpGains();
      if (status != asynSuccess) setIntegerParam(function, oldValue);
    }
    else if (function == AndorCoolerParam) {
      try {
        if (value == 0) {
          asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
            "%s:%s:, CoolerOFF()\n", 
            driverName, functionName);
          checkStatus(CoolerOFF());
        } else if (value == 1) {
          asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
            "%s:%s:, CoolerON()\n", 
            driverName, functionName);
          checkStatus(CoolerON());
        }
      } catch (const std::string &e) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
          "%s:%s: %s\n",
          driverName, functionName, e.c_str());
        status = asynError;
      }
    }
    else if (function == ADShutterControl) {
      status = setupShutter(value);
    }
    else if ((function == ADShutterMode) ||
             (function == AndorShutterMode) ||
             (function == AndorShutterExTTL)) {
      status = setupShutter(-1);
    }
    else if (function == AndorBaselineClamp) {
      try {
        checkStatus(SetBaselineClamp(value));
      } catch (const std::string &e) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
          "%s:%s: %s\n",
          driverName, functionName, e.c_str());
        status = asynError;
      }
    }
    else {
      status = ADDriver::writeInt32(pasynUser, value);
    }

    /* Do callbacks so higher layers see any changes */
    callParamCallbacks();

    /* Send a signal to the poller task which will make it do a poll, and switch to the fast poll rate */
    epicsEventSignal(statusEvent);

    if (mAcquiringData) {
      asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
        "%s:%s:, Sending dataEvent to dataTask ...\n", 
        driverName, functionName);
      // Also signal the data readout thread
      epicsEventSignal(dataEvent);
    }

    if (status) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
              "%s:%s: error, status=%d function=%d, value=%d\n",
              driverName, functionName, status, function, value);
    } else {
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
              "%s:%s: function=%d, value=%d\n",
              driverName, functionName, function, value);
        // For a successful write, clear the error message.
        setStringParam(AndorMessage, " ");
    }
    return status;
}

/** Called when asyn clients call pasynFloat64->write().
  * This function performs actions for some parameters.
  * For all parameters it sets the value in the parameter library and calls any registered callbacks.
  * \param[in] pasynUser pasynUser structure that encodes the reason and address.
  * \param[in] value Value to write. */
asynStatus AndorCCD::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    static const char *functionName = "writeFloat64";

    int minTemp = 0;
    int maxTemp = 0;

    /* Store the old value */
    epicsFloat64 oldValue;
    getDoubleParam(function, &oldValue);

    /* Set the parameter and readback in the parameter library.  */
    status = setDoubleParam(function, value);

    if (function == ADAcquireTime) {
      mAcquireTime = (float)value;  
      status = setupAcquisition();
    }
    else if (function == ADAcquirePeriod) {
      mAcquirePeriod = (float)value;  
      status = setupAcquisition();
    }
    else if (function == AndorAccumulatePeriod) {
      mAccumulatePeriod = (float)value;  
      status = setupAcquisition();
    }
    else if (function == ADTemperature) {
      asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
        "%s:%s:, Setting temperature value %f\n", 
        driverName, functionName, value);
      try {
        /* Check requested temperature is within our range */
        checkStatus(GetTemperatureRange(&minTemp, &maxTemp));
        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
          "%s:%s:, CCD Min Temp: %d, Max Temp %d\n", 
          driverName, functionName, minTemp, maxTemp);
        if ((static_cast<int>(value) >= minTemp) & (static_cast<int>(value) <= maxTemp)) {
          asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
            "%s:%s:, SetTemperature(%d)\n", 
            driverName, functionName, static_cast<int>(value));
          checkStatus(SetTemperature(static_cast<int>(value)));
        } else {
          /* Requested temperature is out of range */
          status = setDoubleParam(function, oldValue);
          asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
            "Requested temperature out of range\n");
          setStringParam(AndorMessage, "Temperature is out of range.");
          callParamCallbacks();
          status = asynError;
        }
      } catch (const std::string &e) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
          "%s:%s: %s\n",
          driverName, functionName, e.c_str());
        status = asynError;
      }
    }
    else if ((function == ADShutterOpenDelay) ||
             (function == ADShutterCloseDelay)) {             
      status = setupShutter(-1);
    }
    else {
      status = ADDriver::writeFloat64(pasynUser, value);
    }

    /* Do callbacks so higher layers see any changes */
    callParamCallbacks();
    if (status) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
              "%s:%s: error, status=%d function=%d, value=%f\n",
              driverName, functionName, status, function, value);
    }
    else {
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
            "%s:%s: function=%d, value=%f\n",
            driverName, functionName, function, value);
        /* For a successful write, clear the error message. */
        setStringParam(AndorMessage, " ");
    }
    return status;
}

/* Called if tracks acquisition mode is being used.
   Sets up the track defintion. */
void AndorCCD::setupTrackDefn(int minX, int sizeX, int binX)
{
    static const char *functionName = "setupTrackDefn";
    if (mMultiTrack.size() == 0)
    {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_WARNING,
            "%s:%s: A track defintion must be set to use tracks mode\n",
            driverName, functionName);
        return;
    }

    static const int ValuesPerTrack = 6;
    std::vector<int> TrackDefn(mMultiTrack.size() * 6);
    setIntegerParam(NDArraySizeY, mMultiTrack.DataHeight());
    for (size_t TrackNo = 0; TrackNo < mMultiTrack.size(); TrackNo++)
    {
        /*
        Each track must be defined by a group of six integers.
            - The top and bottom positions of the tracks.
            - The left and right positions for the area of interest within each track
            - The horizontal and vertical binning for each track. */
        /*
        Andor use 1-based exclusive co-ordinates.
        e.g. from SDK manual:
        1 2 1 1024 1 1
        3 4 1 1024 1 1
        5 6 1 1024 1 1
        7 8 1 1024 1 1
        9 10 1 1024 1 1 */
        TrackDefn[TrackNo * 6 + 0] = mMultiTrack.TrackStart(TrackNo) + 1;
        TrackDefn[TrackNo * 6 + 1] = mMultiTrack.TrackEnd(TrackNo) + 2; // CCDMultiTrack uses 0-based inlcusive co-ordinates.
        TrackDefn[TrackNo * 6 + 2] = minX + 1;
        TrackDefn[TrackNo * 6 + 3] = minX + sizeX;
        TrackDefn[TrackNo * 6 + 4] = binX;
        TrackDefn[TrackNo * 6 + 5] = mMultiTrack.TrackBin(TrackNo);
    }
    checkStatus(SetCustomTrackHBin(binX));
    checkStatus(SetComplexImage(int(TrackDefn.size() / ValuesPerTrack), &TrackDefn[0]));
}

/* Called to set tracks definition parameters.
   Sets up the track defintion. */
asynStatus AndorCCD::writeInt32Array(asynUser *pasynUser, epicsInt32 *value, size_t nElements)
{
    static const char *functionName = "writeInt32Array";
    asynStatus status = asynSuccess;
    try {
        status = mMultiTrack.writeInt32Array(pasynUser, value, nElements);
        if (status != asynError)
            setupAcquisition();
        else
            status = ADDriver::writeInt32Array(pasynUser, value, nElements);
    }
    catch (const std::string &e) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
        "%s:%s: %s\n",
        driverName, functionName, e.c_str());
        status = asynError;
    }
    return status;
}

/** Controls shutter
 * @param[in] command 0=close, 1=open, -1=no change, only set other parameters */
asynStatus AndorCCD::setupShutter(int command)
{
  double dTemp;
  int openTime, closeTime;
  int shutterExTTL;
  int adShutterMode;
  int shutterMode;
  asynStatus status=asynSuccess;
  static const char *functionName = "setupShutter";

  getIntegerParam(ADShutterMode, &adShutterMode);
  if (adShutterMode == ADShutterModeNone) return asynSuccess;
  if ((adShutterMode == ADShutterModeEPICS) && (command != -1)) {
    ADDriver::setShutter(command);
    return asynSuccess;
  }

  /* We are using internal shutter mode */
  getDoubleParam(ADShutterOpenDelay, &dTemp);
  // Convert to ms
  openTime = (int)(dTemp * 1000.);
  if (openTime < mMinShutterOpenTime) {
    openTime = mMinShutterOpenTime;
    setDoubleParam(ADShutterOpenDelay, openTime / 1000.);
  }
  getDoubleParam(ADShutterCloseDelay, &dTemp);
  closeTime = (int)(dTemp * 1000.);
  if (closeTime < mMinShutterCloseTime) {
    closeTime = mMinShutterCloseTime;
    setDoubleParam(ADShutterCloseDelay, closeTime / 1000.);
  }
  getIntegerParam(AndorShutterMode, &shutterMode);
  getIntegerParam(AndorShutterExTTL, &shutterExTTL);
  
  if (command == ADShutterClosed) {
    shutterMode = AShutterAlwaysClosed;
    setIntegerParam(ADShutterStatus, ADShutterClosed);
  }
  else if (command == ADShutterOpen) {
    if (shutterMode == AShutterAlwaysOpen) {
      setIntegerParam(ADShutterStatus, ADShutterOpen);
    }
    // No need to change shutterMode, we leave it alone and it shutter
    // will do correct thing, i.e. auto or open depending shutterMode
  }

  try {
    if (mCapabilities.ulFeatures & AC_FEATURES_SHUTTER) {
      asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
        "%s:%s:, SetShutter(%d,%d,%d,%d)\n",
        driverName, functionName, shutterExTTL, shutterMode, closeTime, openTime);
      checkStatus(SetShutter(shutterExTTL, shutterMode, closeTime, openTime));
    }
  } catch (const std::string &e) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
      "%s:%s: %s\n",
      driverName, functionName, e.c_str());
    status = asynError;
  }
  return status;
}



/**
 * Function to check the return status of Andor SDK library functions.
 * @param returnStatus The return status of the SDK function
 * @return 0=success. Does not return in case of failure.
 * @throw std::string An exception is thrown in case of failure.
 */
unsigned int AndorCCD::checkStatus(unsigned int returnStatus)
{
  char message[256];
  if (returnStatus == DRV_SUCCESS) {
    return 0;
  } else if (returnStatus == DRV_NOT_INITIALIZED) {
    throw std::string("ERROR: Driver is not initialized.");
  } else if (returnStatus == DRV_ACQUIRING) {
    throw std::string("ERROR: Not allowed. Currently acquiring data.");
  } else if (returnStatus == DRV_P1INVALID) {
    throw std::string("ERROR: Parameter 1 not valid.");
  } else if (returnStatus == DRV_P2INVALID) {
    throw std::string("ERROR: Parameter 2 not valid.");
  } else if (returnStatus == DRV_P3INVALID) {
    throw std::string("ERROR: Parameter 3 not valid.");
  } else if (returnStatus == DRV_P4INVALID) {
    throw std::string("ERROR: Parameter 4 not valid.");
  } else if (returnStatus == DRV_P5INVALID) {
    throw std::string("ERROR: Parameter 5 not valid.");
  } else if (returnStatus == DRV_P6INVALID) {
    throw std::string("ERROR: Parameter 6 not valid.");
  } else if (returnStatus == DRV_P7INVALID) {
    throw std::string("ERROR: Parameter 7 not valid.");
  } else if (returnStatus == DRV_ERROR_ACK) {
    throw std::string("ERROR: Unable to communicate with card.");
  } else if (returnStatus == DRV_TEMP_OFF) {
    setStringParam(AndorTempStatusMessage, "Cooler is OFF");
    return 0;
  } else if (returnStatus == DRV_TEMP_STABILIZED) {
    setStringParam(AndorTempStatusMessage, "Stabilized at set point");
    return 0;
  } else if (returnStatus == DRV_TEMP_NOT_REACHED) {
    setStringParam(AndorTempStatusMessage, "Not reached setpoint");
    return 0;
  } else if (returnStatus == DRV_TEMP_DRIFT) {
    setStringParam(AndorTempStatusMessage, "Stabilized but drifted");
    return 0;
  } else if (returnStatus == DRV_TEMP_NOT_STABILIZED) {
    setStringParam(AndorTempStatusMessage, "Not stabilized at set point");
    return 0;
  } else if (returnStatus == DRV_VXDNOTINSTALLED) {
    throw std::string("ERROR: VxD not loaded.");
  } else if (returnStatus == DRV_INIERROR) {
    throw std::string("ERROR: Unable to load DETECTOR.INI.");
  } else if (returnStatus == DRV_COFERROR) {
    throw std::string("ERROR: Unable to load *.COF.");
  } else if (returnStatus == DRV_FLEXERROR) {
    throw std::string("ERROR: Unable to load *.RBF.");
  } else if (returnStatus == DRV_ERROR_FILELOAD) {
    throw std::string("ERROR: Unable to load *.COF or *.RBF files.");
  } else if (returnStatus == DRV_ERROR_PAGELOCK) {
    throw std::string("ERROR: Unable to acquire lock on requested memory.");
  } else if (returnStatus == DRV_USBERROR) {
    throw std::string("ERROR: Unable to detect USB device or not USB 2.0.");
  } else if (returnStatus == DRV_ERROR_NOCAMERA) {
    throw std::string("ERROR: No camera found.");
  } else if (returnStatus == DRV_GENERAL_ERRORS) {
    throw std::string("ERROR: An error occured while obtaining the number of available cameras.");
  } else if (returnStatus == DRV_INVALID_MODE) {
    throw std::string("ERROR: Invalid mode or mode not available.");
  } else if (returnStatus == DRV_ACQUISITION_ERRORS) {
    throw std::string("ERROR: Acquisition mode are invalid.");
  } else if (returnStatus == DRV_ERROR_PAGELOCK) {
    throw std::string("ERROR: Unable to allocate memory.");
  } else if (returnStatus == DRV_INVALID_FILTER) {
    throw std::string("ERROR: Filter not available for current acquisition.");
  } else if (returnStatus == DRV_IDLE) {
    throw std::string("ERROR: The system is not currently acquiring.");  
  } else if (returnStatus == DRV_NO_NEW_DATA) {
    throw std::string("ERROR: No data to read, or CancelWait() called.");  
  } else if (returnStatus == DRV_ERROR_CODES) {
    throw std::string("ERROR: Problem communicating with camera.");  
  } else if (returnStatus == DRV_LOAD_FIRMWARE_ERROR) {
    throw std::string("ERROR: Error loading firmware.");  
  } else if (returnStatus == DRV_NOT_SUPPORTED) {
    throw std::string("ERROR: Feature not supported.");
  } else if (returnStatus == DRV_RANDOM_TRACK_ERROR) {
    throw std::string("ERROR: Invalid combination of tracks");
  } else {
    sprintf(message, "ERROR: Unknown error code=%d returned from Andor SDK.", returnStatus);
    throw std::string(message);
  }

  return 0;
}


/**
 * Update status of detector. Meant to be run in own thread.
 */
void AndorCCD::statusTask(void)
{
  int value = 0;
  float temperature;
  unsigned int uvalue = 0;
  unsigned int status = 0;
  double timeout = 0.0;
  unsigned int forcedFastPolls = 0;
  static const char *functionName = "statusTask";

  printf("%s:%s: Status thread started...\n", driverName, functionName);
  while(!mExiting) {

    // Read timeout for polling freq.
    this->lock();
    if (forcedFastPolls > 0) {
      timeout = mFastPollingPeriod;
      forcedFastPolls--;
    } else {
      timeout = mPollingPeriod;
    }
    this->unlock();

    if (timeout != 0.0) {
      status = epicsEventWaitWithTimeout(statusEvent, timeout);
    } else {
      status = epicsEventWait(statusEvent);
    }
    if (status == epicsEventWaitOK) {
      asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
        "%s:%s: Got status event\n",
        driverName, functionName);
      // We got an event, rather than a timeout.  This is because other software
      // knows that data has arrived, or device should have changed state (parameters changed, etc.).
      // Force a minimum number of fast polls, because the device status
      // might not have changed in the first few polls
      forcedFastPolls = 5;
    }

    if (mExiting) break;
    this->lock();

    try {
      // Only read these if we are not acquiring data
      if (!mAcquiringData) {
        // Read cooler status
        checkStatus(IsCoolerOn(&value));
        status = setIntegerParam(AndorCoolerParam, value);
        // Read temperature of CCD
        checkStatus(GetTemperatureF(&temperature));
        status = setDoubleParam(ADTemperatureActual, temperature);
      }

      // Read detector status (idle, acquiring, error, etc.)
      checkStatus(GetStatus(&value));
      uvalue = static_cast<unsigned int>(value);
      if (uvalue == ASIdle) {
        setIntegerParam(ADStatus, ADStatusIdle);
        setStringParam(ADStatusMessage, "IDLE. Waiting on instructions.");
      } else if (uvalue == ASTempCycle) {
        setIntegerParam(ADStatus, ADStatusWaiting);
        setStringParam(ADStatusMessage, "Executing temperature cycle.");
      } else if (uvalue == ASAcquiring) {
        setIntegerParam(ADStatus, ADStatusAcquire);
        setStringParam(ADStatusMessage, "Data acquisition in progress.");
      } else if (uvalue == ASAccumTimeNotMet) {
        setIntegerParam(ADStatus, ADStatusError);
        setStringParam(ADStatusMessage, "Unable to meet accumulate time.");
      } else if (uvalue == ASKineticTimeNotMet) {
        setIntegerParam(ADStatus, ADStatusError);
        setStringParam(ADStatusMessage, "Unable to meet kinetic cycle time.");
      } else if (uvalue == ASErrorAck) {
        setIntegerParam(ADStatus, ADStatusError);
        setStringParam(ADStatusMessage, "Unable to communicate with device.");
      } else if (uvalue == ASAcqBuffer) {
        setIntegerParam(ADStatus, ADStatusError);
        setStringParam(ADStatusMessage, "Computer unable to read data from device at required rate.");
      } else if (uvalue == ASSpoolError) {
        setIntegerParam(ADStatus, ADStatusError);
        setStringParam(ADStatusMessage, "Overflow of the spool buffer.");
      }
    } catch (const std::string &e) {
      asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
        "%s:%s: %s\n",
        driverName, functionName, e.c_str());
      setStringParam(AndorMessage, e.c_str());
    }

    /* Call the callbacks to update any changes */
    callParamCallbacks();
    this->unlock();
        
  } // End of loop
  asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
      "%s:%s: Status thread exiting ...\n",
      driverName, functionName);

  mExited++;
}

/** Set up acquisition parameters */
asynStatus AndorCCD::setupAcquisition()
{
  int numExposures;
  int numImages;
  int imageMode;
  int adcSpeed;
  int triggerMode;
  int preAmpGain;
  int binX, binY, minX, minY, sizeX, sizeY, reverseX, reverseY, maxSizeX, maxSizeY;
  float acquireTimeAct, acquirePeriodAct, accumulatePeriodAct;
  int FKmode = 4;
  int emGain;
  int emGainMode;
  int emGainAdvanced;
  int FKOffset;
  AndorADCSpeed_t *pSpeed;
  int readOutMode;
  int frameTransferMode;
  int verticalShiftPeriod;
  int verticalShiftAmplitude;
  static const char *functionName = "setupAcquisition";
  
  if (!mInitOK) {
    return asynDisabled;
  }

  // Get current readout mode
  getIntegerParam(AndorReadOutMode, &readOutMode);
  getIntegerParam(ADImageMode, &imageMode);
  getIntegerParam(ADNumExposures, &numExposures);
  if (numExposures <= 0) {
    numExposures = 1;
    setIntegerParam(ADNumExposures, numExposures);
  }
  getIntegerParam(ADNumImages, &numImages);
  if (numImages <= 0) {
    numImages = 1;
    setIntegerParam(ADNumImages, numImages);
  }
  getIntegerParam(ADBinX, &binX);
  if (binX <= 0) {
    binX = 1;
    setIntegerParam(ADBinX, binX);
  }
  getIntegerParam(ADBinY, &binY);
  if (binY <= 0) {
    binY = 1;
    setIntegerParam(ADBinY, binY);
  }
  // Check EM gain capability and range, and set gain mode before setting gain and limits
  if ((int)mCapabilities.ulEMGainCapability > 0) {
    getIntegerParam(AndorEmGainAdvanced, &emGainAdvanced);
    setIntegerParam(AndorEmGainAdvanced, emGainAdvanced);
    getIntegerParam(AndorEmGainMode, &emGainMode);
    setIntegerParam(AndorEmGainMode, emGainMode);
    checkStatus(GetEMGainRange(&mEmGainRangeLow, &mEmGainRangeHigh));
    getIntegerParam(AndorEmGain, &emGain);
    if (emGain < mEmGainRangeLow) {
      emGain = mEmGainRangeLow;
      setIntegerParam(AndorEmGain, emGain);
    }
    else if (emGain > mEmGainRangeHigh) {
      emGain = mEmGainRangeHigh;
      setIntegerParam(AndorEmGain, emGain);
    }
  }
  getIntegerParam(ADMinX, &minX);
  getIntegerParam(ADMinY, &minY);
  getIntegerParam(ADSizeX, &sizeX);
  getIntegerParam(ADSizeY, &sizeY);
  getIntegerParam(ADReverseX, &reverseX);
  getIntegerParam(ADReverseY, &reverseY);
  getIntegerParam(ADMaxSizeX, &maxSizeX);
  getIntegerParam(ADMaxSizeY, &maxSizeY);
  if (readOutMode == ARFullVerticalBinning) {
    // Set maximum binning but do not update parameter, this preserves ADBinY
    // when going back to Image readout mode.
    binY = maxSizeY;
  }
  if (minX > (maxSizeX - binX)) {
    minX = maxSizeX - binX;
    setIntegerParam(ADMinX, minX);
  }
  if (minY > (maxSizeY - binY)) {
    minY = maxSizeY - binY;
    setIntegerParam(ADMinY, minY);
  }
  if ((minX + sizeX) > maxSizeX) {
    sizeX = maxSizeX - minX;
    setIntegerParam(ADSizeX, sizeX);
  }
  if ((minY + sizeY) > maxSizeY) {
    sizeY = maxSizeY - minY;
    setIntegerParam(ADSizeY, sizeY);
  }
  
  // Note: we do triggerMode and adcChannel in this function because they change
  // the computed actual AcquirePeriod and AccumulatePeriod
  getIntegerParam(ADTriggerMode, &triggerMode);
  getIntegerParam(AndorAdcSpeed, &adcSpeed);
  pSpeed = &mADCSpeeds[adcSpeed];
  
  getIntegerParam(AndorPreAmpGain, &preAmpGain);
  
  // Unfortunately there does not seem to be a way to query the Andor SDK 
  // for the actual size of the image, so we must compute it.
  setIntegerParam(NDArraySizeX, sizeX/binX);
  if (readOutMode != ARRandomTrack)
      // The data height dimension is set by setupTrackDefn for multi-track mode.
    setIntegerParam(NDArraySizeY, sizeY/binY);

  getIntegerParam(AndorFrameTransferMode, &frameTransferMode);

  getIntegerParam(AndorVerticalShiftPeriod, &verticalShiftPeriod);

  getIntegerParam(AndorVerticalShiftAmplitude, &verticalShiftAmplitude);

  try {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
      "%s:%s:, SetReadMode(%d)\n",
      driverName, functionName, readOutMode);
    checkStatus(SetReadMode(readOutMode));
    if (readOutMode == ARRandomTrack)
        setupTrackDefn(minX, sizeX, binX);

    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
      "%s:%s:, SetTriggerMode(%d)\n", 
      driverName, functionName, triggerMode);
    checkStatus(SetTriggerMode(triggerMode));

    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
      "%s:%s:, SetADChannel(%d)\n", 
      driverName, functionName, pSpeed->ADCIndex);
    checkStatus(SetADChannel(pSpeed->ADCIndex));

    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
      "%s:%s:, SetOutputAmplifier(%d)\n", 
      driverName, functionName, pSpeed->AmpIndex);
    checkStatus(SetOutputAmplifier(pSpeed->AmpIndex));

    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
      "%s:%s:, SetHSSpeed(%d, %d)\n", 
      driverName, functionName, pSpeed->AmpIndex, pSpeed->HSSpeedIndex);
    checkStatus(SetHSSpeed(pSpeed->AmpIndex, pSpeed->HSSpeedIndex));

    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
      "%s:%s:, SetPreAmpGain(%d)\n", 
      driverName, functionName, preAmpGain);
    checkStatus(SetPreAmpGain(preAmpGain));

    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
      "%s:%s:, SetImageFlip(%d, %d)\n", 
      driverName, functionName, reverseX, reverseY);
    checkStatus(SetImageFlip(reverseX, reverseY));

    if (readOutMode == ARImage) {
      asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
        "%s:%s:, SetImage(%d,%d,%d,%d,%d,%d)\n",
        driverName, functionName, binX, binY, minX+1, minX+sizeX, minY+1, minY+sizeY);
      checkStatus(SetImage(binX, binY, minX+1, minX+sizeX, minY+1, minY+sizeY));
    }

    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
      "%s:%s:, SetExposureTime(%f)\n", 
      driverName, functionName, mAcquireTime);
    checkStatus(SetExposureTime(mAcquireTime));

    // Check if camera has EM gain capability before setting modes or EM gain
    if ((int)mCapabilities.ulEMGainCapability > 0) {
      asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
        "%s:%s:, SetEMGainMode(%d)\n", 
        driverName, functionName, emGainMode);
      checkStatus(SetEMGainMode(emGainMode));
    }

    if ((int)mCapabilities.ulEMGainCapability > 0) {
      asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
        "%s:%s:, SetEMGainAdvanced(%d)\n", 
        driverName, functionName, emGainAdvanced);
      checkStatus(SetEMAdvanced(emGainAdvanced));
    }

    if ((int)mCapabilities.ulEMGainCapability > 0) {
      asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
        "%s:%s:, SetEMCCDGain(%d)\n", 
        driverName, functionName, emGain);
      checkStatus(SetEMCCDGain(emGain));
    }

    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
      "%s:%s:, SetFrameTransferMode(%d)\n",
      driverName, functionName, frameTransferMode);
    checkStatus(SetFrameTransferMode(frameTransferMode));

    float s = 0.0f;
    unsigned int error = GetVSSpeed(0, &s);
    if (error == DRV_SUCCESS ) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
            "%s:%s:, SetVSSpeed(%d)\n",
            driverName, functionName, verticalShiftPeriod);
        checkStatus(SetVSSpeed(verticalShiftPeriod));
    }

    if ((mCapabilities.ulSetFunctions & AC_SETFUNCTION_VSAMPLITUDE) != 0)
    {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
          "%s:%s:, SetVSAmplitude(%d)\n",
          driverName, functionName, verticalShiftAmplitude);
        checkStatus(SetVSAmplitude(verticalShiftAmplitude));
    }

    switch (imageMode) {
      case ADImageSingle:
        if (numExposures == 1) {
          asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
            "%s:%s:, SetAcquisitionMode(AASingle)\n", 
            driverName, functionName);
          checkStatus(SetAcquisitionMode(AASingle));
        } else {
          asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
            "%s:%s:, SetAcquisitionMode(AAAccumulate)\n", 
            driverName, functionName);
          checkStatus(SetAcquisitionMode(AAAccumulate));
          asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
            "%s:%s:, SetNumberAccumulations(%d)\n", 
            driverName, functionName, numExposures);
          checkStatus(SetNumberAccumulations(numExposures));
          asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
            "%s:%s:, SetAccumulationCycleTime(%f)\n", 
            driverName, functionName, mAccumulatePeriod);
          checkStatus(SetAccumulationCycleTime(mAccumulatePeriod));
        }
        break;

      case ADImageMultiple:
        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
          "%s:%s:, SetAcquisitionMode(AAKinetics)\n", 
          driverName, functionName);
        checkStatus(SetAcquisitionMode(AAKinetics));
        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
          "%s:%s:, SetNumberAccumulations(%d)\n", 
          driverName, functionName, numExposures);
        checkStatus(SetNumberAccumulations(numExposures));
        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
          "%s:%s:, SetAccumulationCycleTime(%f)\n", 
          driverName, functionName, mAccumulatePeriod);
        checkStatus(SetAccumulationCycleTime(mAccumulatePeriod));
        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
          "%s:%s:, SetNumberKinetics(%d)\n", 
          driverName, functionName, numImages);
        checkStatus(SetNumberKinetics(numImages));
        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
          "%s:%s:, SetKineticCycleTime(%f)\n", 
          driverName, functionName, mAcquirePeriod);
        checkStatus(SetKineticCycleTime(mAcquirePeriod));
        break;

      case ADImageContinuous:
        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
          "%s:%s:, SetAcquisitionMode(AARunTillAbort)\n", 
          driverName, functionName);
        checkStatus(SetAcquisitionMode(AARunTillAbort));
        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
          "%s:%s:, SetKineticCycleTime(%f)\n", 
          driverName, functionName, mAcquirePeriod);
        checkStatus(SetKineticCycleTime(mAcquirePeriod));
        break;

      case AImageFastKinetics:
        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
          "%s:%s:, SetAcquisitionMode(AAFastKinetics)\n", 
          driverName, functionName);
        checkStatus(SetAcquisitionMode(AAFastKinetics));
        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
          "%s:%s:, SetImage(%d,%d,%d,%d,%d,%d)\n", 
          driverName, functionName, binX, binY, 1, maxSizeX, 1, maxSizeY);
        checkStatus(SetImage(binX, binY, 1, maxSizeX, 1, maxSizeY));
        FKOffset = maxSizeY - sizeY - minY;
        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
          "%s:%s:, SetFastKineticsEx(%d,%d,%f,%d,%d,%d,%d)\n", 
          driverName, functionName, sizeY, numImages, mAcquireTime, FKmode, binX, binY, FKOffset);
        checkStatus(SetFastKineticsEx(sizeY, numImages, mAcquireTime, FKmode, binX, binY, FKOffset));
        setIntegerParam(NDArraySizeX, maxSizeX/binX);
        setIntegerParam(NDArraySizeY, sizeY/binY);
        break;
    }
    // Read the actual times
    if (imageMode == AImageFastKinetics) {
      checkStatus(GetFKExposureTime(&acquireTimeAct));
    } else {
      checkStatus(GetAcquisitionTimings(&acquireTimeAct, &accumulatePeriodAct, &acquirePeriodAct));
      asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
        "%s:%s:, GetAcquisitionTimings(exposure=%f, accumulate=%f, kinetic=%f)\n",
        driverName, functionName, acquireTimeAct, accumulatePeriodAct, acquirePeriodAct);
    }
    setDoubleParam(ADAcquireTime, acquireTimeAct);
    setDoubleParam(ADAcquirePeriod, acquirePeriodAct);
    setDoubleParam(AndorAccumulatePeriod, accumulatePeriodAct);

    
  } catch (const std::string &e) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
      "%s:%s: %s\n",
      driverName, functionName, e.c_str());
    return asynError;
  }
  return asynSuccess;
}


/**
 * Do data readout from the detector. Meant to be run in own thread.
 */
void AndorCCD::dataTask(void)
{
  epicsUInt32 status = 0;
  int acquireStatus;
  char *errorString = NULL;
  int acquiring = 0;
  epicsInt32 numImagesCounter;
  epicsInt32 numExposuresCounter;
  epicsInt32 imageCounter;
  epicsInt32 arrayCallbacks;
  epicsInt32 sizeX, sizeY;
  int adShutterMode;
  NDDataType_t dataType;
  int itemp;
  at_32 firstImage, lastImage;
  at_32 validFirst, validLast;
  size_t dims[2];
  int nDims = 2;
  int i;
  NDArray *pArray;
  int autoSave;
  int readOutMode;
  static const char *functionName = "dataTask";

  printf("%s:%s: Data thread started...\n", driverName, functionName);

  this->lock();

  while(!mExiting) {
    
    errorString = NULL;

    // Wait for event from main thread to signal that data acquisition has started.
    this->unlock();
    status = epicsEventWait(dataEvent);
    if (mExiting)
        break;
    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
      "%s:%s:, got data event\n", 
      driverName, functionName);
    this->lock();

    // Sanity check that main thread thinks we are acquiring data
    if (mAcquiringData) {
      // Read some parameters
      getIntegerParam(ADShutterMode, &adShutterMode);
      getIntegerParam(AndorReadOutMode, &readOutMode);
      getIntegerParam(NDDataType, &itemp); dataType = (NDDataType_t)itemp;
      getIntegerParam(NDAutoSave, &autoSave);
      getIntegerParam(NDArrayCallbacks, &arrayCallbacks);
      getIntegerParam(NDArraySizeX, &sizeX);
      getIntegerParam(NDArraySizeY, &sizeY);
      // Set acquiring to 1
      acquiring = 1;
    } else {
      asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
        "%s:%s:, Data thread is running but main thread thinks we are not acquiring.\n", 
        driverName, functionName);
      // Set acquiring to 0
      acquiring = 0;
    }

    while ((acquiring) && (!mExiting)) {
      try {
        checkStatus(GetStatus(&acquireStatus));
        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
          "%s:%s:, GetStatus returned %d\n",
          driverName, functionName, acquireStatus);
        if (acquireStatus != DRV_ACQUIRING) break;
        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
          "%s:%s:, WaitForAcquisition().\n",
          driverName, functionName);
        this->unlock();
        checkStatus(WaitForAcquisition());
        this->lock();
        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
          "%s:%s:, WaitForAcquisition has returned.\n",
          driverName, functionName);
        getIntegerParam(ADNumExposuresCounter, &numExposuresCounter);
        numExposuresCounter++;
        setIntegerParam(ADNumExposuresCounter, numExposuresCounter);
        callParamCallbacks();
        // Is there an image available?
        status = GetNumberNewImages(&firstImage, &lastImage);
        if (status != DRV_SUCCESS) continue;
        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
          "%s:%s:, firstImage=%ld, lastImage=%ld\n",
          driverName, functionName, (long)firstImage, (long)lastImage);
        for (i=firstImage; i<=lastImage; i++) {
          // Update counters
          getIntegerParam(NDArrayCounter, &imageCounter);
          imageCounter++;
          setIntegerParam(NDArrayCounter, imageCounter);;
          getIntegerParam(ADNumImagesCounter, &numImagesCounter);
          numImagesCounter++;
          setIntegerParam(ADNumImagesCounter, numImagesCounter);
          // If array callbacks are enabled then read data into NDArray, do callbacks
          if (arrayCallbacks) {
            // Allocate an NDArray
            dims[0] = sizeX;
            dims[1] = sizeY;
            pArray = this->pNDArrayPool->alloc(nDims, dims, dataType, 0, NULL);
            if (readOutMode == ARRandomTrack)
                mMultiTrack.storeTrackAttributes(pArray->pAttributeList);
            // Read the oldest array
            // Is there still an image available?
            status = GetNumberNewImages(&firstImage, &lastImage);
            asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
              "%s:%s:, GetNumberNewImages, status=%d, firstImage=%ld, lastImage=%ld\n", 
              driverName, functionName, status, (long)firstImage, (long)lastImage);
            if (dataType == NDUInt32) {
              asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
                "%s:%s:, GetImages(%d, %d, %p, %d, %p, %p)\n", 
                driverName, functionName, i, i, pArray->pData, sizeX*sizeY, &validFirst, &validLast);
              checkStatus(GetImages(i, i, (at_32*)pArray->pData, 
                                    sizeX*sizeY, &validFirst, &validLast));
              setIntegerParam(NDArraySize, sizeX * sizeY * sizeof(epicsUInt32));
            }
            else if (dataType == NDUInt16) {
              asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
                "%s:%s:, GetImages16(%d, %d, %p, %d, %p, %p)\n", 
                driverName, functionName, i, i, pArray->pData, sizeX*sizeY, &validFirst, &validLast);
              checkStatus(GetImages16(i, i, (epicsUInt16*)pArray->pData, 
                                      sizeX*sizeY, &validFirst, &validLast));
              setIntegerParam(NDArraySize, sizeX * sizeY * sizeof(epicsUInt16));
            }
            /* Put the frame number and time stamp into the buffer */
            pArray->uniqueId = imageCounter;
            updateTimeStamps(pArray);
            /* Get any attributes that have been defined for this driver */        
            this->getAttributes(pArray->pAttributeList);
            /* Call the NDArray callback */
            asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
                 "%s:%s:, calling array callbacks\n", 
                 driverName, functionName);
            doCallbacksGenericPointer(pArray, NDArrayData, 0);
            // Save the current frame for use with the SPE file writer which needs the data
            if (this->pArrays[0]) this->pArrays[0]->release();
            this->pArrays[0] = pArray;
          }
          // Save data if autosave is enabled
          if (autoSave) this->saveDataFrame(i);
          callParamCallbacks();
        }
      } catch (const std::string &e) {
          if (!mExiting)
          {
              asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:%s: %s\n",
                  driverName, functionName, e.c_str());
              errorString = const_cast<char *>(e.c_str());
              setStringParam(AndorMessage, errorString);
          }
      }
    }
    
    // Close the shutter if we are controlling it
    if (adShutterMode == ADShutterModeEPICS) {
      ADDriver::setShutter(ADShutterClosed);
    }
    
    // Wait for the status thread to set ADStatus to something other than Acquiring
    while (1) {
      epicsInt32 acquireStatus;
      getIntegerParam(ADStatus,&acquireStatus);
      if (acquireStatus!=ADStatusAcquire)  break;
      // Allow other threads to update ADStatusAcquire
      this->unlock();
      epicsThreadSleep(0.01);
      this->lock();
    }

    // Now clear main thread flag
    mAcquiringData = 0;
    setIntegerParam(ADAcquire, 0);

    /* Call the callbacks to update any changes */
    callParamCallbacks();
  } // End of loop
  mExited++;
  this->unlock();
}


/**
 * Save a data frame using the Andor SDK file writing functions.
 */
void AndorCCD::saveDataFrame(int frameNumber) 
{
  char *errorString = NULL;
  int fileFormat;
  NDDataType_t dataType;
  int itemp;
  int FITSType=0;
  char fullFileName[MAX_FILENAME_LEN];
  char palFilePath[MAX_FILENAME_LEN];
  static const char *functionName = "saveDataFrame";

  // Fetch the file format
  getIntegerParam(NDFileFormat, &fileFormat);
      
  this->createFileName(255, fullFileName);
  setStringParam(NDFullFileName, fullFileName);
  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
    "%s:%s:, file name is %s.\n",
    driverName, functionName, fullFileName);
  getStringParam(AndorPalFileName, 255, palFilePath);

  try {
    if (fileFormat == AFFTIFF) {
      asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
        "%s:%s:, SaveAsTiffEx(%s, %s, %d, 1, 1)\n", 
        driverName, functionName, fullFileName, palFilePath, frameNumber);
      checkStatus(SaveAsTiffEx(fullFileName, palFilePath, frameNumber, 1, 1));
    } else if (fileFormat == AFFBMP) {
      asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
        "%s:%s:, SaveAsBmp(%s, %s, 0, 0)\n", 
        driverName, functionName, fullFileName, palFilePath);
      checkStatus(SaveAsBmp(fullFileName, palFilePath, 0, 0));
    } else if (fileFormat == AFFSIF) {
      asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
        "%s:%s:, SaveAsSif(%s)\n", 
        driverName, functionName, fullFileName);
      checkStatus(SaveAsSif(fullFileName));
    } else if (fileFormat == AFFEDF) {
      asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
        "%s:%s:, SaveAsEDF(%s, 0)\n", 
        driverName, functionName, fullFileName);
      checkStatus(SaveAsEDF(fullFileName, 0));
    } else if (fileFormat == AFFRAW) {
      asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
        "%s:%s:, SaveAsRaw(%s, 1)\n", 
        driverName, functionName, fullFileName);
      checkStatus(SaveAsRaw(fullFileName, 1));
    } else if (fileFormat == AFFFITS) {
      getIntegerParam(NDDataType, &itemp); dataType = (NDDataType_t)itemp;
      if (dataType == NDUInt16) FITSType=0;
      else if (dataType== NDUInt32) FITSType=1;
      asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
        "%s:%s:, SaveAsFITS(%s, %d)\n", 
        driverName, functionName, fullFileName, FITSType);
      checkStatus(SaveAsFITS(fullFileName, FITSType));
    } else if (fileFormat == AFFSPE) {
      asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
        "%s:%s:, SaveAsSPE(%s)\n", 
        driverName, functionName, fullFileName);
      checkStatus(SaveAsSPE(fullFileName));
    }
  } catch (const std::string &e) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
      "%s:%s: %s\n",
      driverName, functionName, e.c_str());
    errorString = const_cast<char *>(e.c_str());
  }

  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
    "%s:%s:, Saving data.. Done!\n", 
    driverName, functionName);

  if (errorString != NULL) {
    setStringParam(AndorMessage, errorString);
    setIntegerParam(ADStatus, ADStatusError);
  }

}

xmlNode *xmlFindChildElement(xmlNode *parent, const char *name)
{
  xmlNode *node;
  for (node = xmlFirstElementChild(parent); node; node = xmlNextElementSibling(parent)) {
    if ((xmlStrEqual(node->name, (const xmlChar *)name))) {
      return node;
    }
  }
  return 0;
}

unsigned int AndorCCD::SaveAsSPE(char *fullFileName)
{
  NDArray *pArray = this->pArrays[0];
  NDArrayInfo arrayInfo;
  int nx, ny;
  bool xmlError;
  int dataType;
  FILE *fp;
  size_t numWrite;
  float *calibration;
  char *calibrationString;
  char tempString[20];
  const char *dataTypeString;
  int i;
  xmlNode *speFormatElement, *dataFormatElement, *calibrationsElement;
  xmlNode *wavelengthMappingElement, *wavelengthElement;
  xmlNode *dataBlockElement, *dataBlockElement2;
  xmlNode *sensorInformationElement, *sensorMappingElement;
  static const char *functionName="SaveAsSPE";
  
  if (!pArray) return DRV_NO_NEW_DATA;
  pArray->getInfo(&arrayInfo);
  nx = (int) arrayInfo.xSize;
  ny = (int) arrayInfo.ySize;  
  
  // Fill in the SPE file header
  mSPEHeader->xdim = nx;
  mSPEHeader->ydim = ny;
  if (pArray->dataType == NDUInt16) {
    dataTypeString = "MonochromeUnsigned16";
    dataType = 3;
  } else if (pArray->dataType == NDUInt32) {
    dataTypeString = "MonochromeUnsigned32";
    dataType = 1;
  } else {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
      "%s::%s error unknown data type %d\n",
      driverName, functionName, pArray->dataType);
    return DRV_GENERAL_ERRORS;
  }
  
  mSPEHeader->datatype = dataType;
  mSPEHeader->scramble = 1;
  mSPEHeader->lnoscan  = -1;
  mSPEHeader->NumExpAccums  = 1;
  mSPEHeader->NumFrames  = 1;
  mSPEHeader->file_header_ver  = 3.0;
  mSPEHeader->WinView_id = 0x01234567;
  mSPEHeader->lastvalue = 0x5555;
  mSPEHeader->XML_Offset = sizeof(*mSPEHeader) + arrayInfo.totalBytes;
  
  // Open the file
  fp = fopen(fullFileName, "wb");
  if (!fp) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
      "%s::%s error opening file %s error=%s\n",
      driverName, functionName, fullFileName, strerror(errno));
    return DRV_GENERAL_ERRORS;
  }
  
  // Write the header to the file
  numWrite = fwrite(mSPEHeader, sizeof(*mSPEHeader), 1, fp);
  if (numWrite != 1) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
      "%s::%s error writing SPE file header\n",
      driverName, functionName);
    fclose(fp);
    return DRV_GENERAL_ERRORS;
  }
  
  // Write the data to the file
  numWrite = fwrite(pArray->pData, arrayInfo.totalBytes, 1, fp);
  if (numWrite != 1) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
      "%s::%s error writing SPE data\n",
      driverName, functionName);
    fclose(fp);
    return DRV_GENERAL_ERRORS;
  }

  // Create the default calibration
  calibration = (float *) calloc(nx, sizeof(float));
  calibrationString = (char *) calloc(nx*20, sizeof(char));
  for (i=0; i<nx; i++) calibration[i] = (float) i; 
  
  // If there is a valid Shamrock spectrometer get the calibration
  #ifdef USE_SHAMROCK
    int error;
    int numSpectrometers;
    error = ShamrockGetNumberDevices(&numSpectrometers);
    if (error != SHAMROCK_SUCCESS) goto noSpectrometers;
    if (numSpectrometers < 1) goto noSpectrometers;
    if ((mShamrockId < 0) || (mShamrockId > numSpectrometers-1)) goto noSpectrometers;
    error = ShamrockGetCalibration(mShamrockId, calibration, nx);
    if (error != SHAMROCK_SUCCESS) {
      asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
        "%s::%s error reading Shamrock spectrometer calibration\n",
        driverName, functionName);
    }
   
    // Create the calibration string
    for (i=0; i<nx; i++) {
      if (i > 0) strcat(calibrationString, ",");
      sprintf(tempString, "%.6f", calibration[i]);
      strcat(calibrationString, tempString);
    }
  noSpectrometers:  
  #endif //USE_SHAMROCK

  // Create the XML data using SPETemplate.xml in the current directory as a template    
  if (mSPEDoc == 0) {
    mSPEDoc = xmlReadFile("SPETemplate.xml", NULL, 0);
    if (mSPEDoc == 0) {
      asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
        "%s::%s error opening SPETemplate.xml\n",
        driverName, functionName);
      return DRV_GENERAL_ERRORS;
    }
  }
  
  // Assume XML parsing error
  xmlError = true;
      
  // Set the required values in the DataFormat element
  speFormatElement = xmlDocGetRootElement(mSPEDoc);
  if ((!xmlStrEqual(speFormatElement->name, (const xmlChar *)"SpeFormat"))) {
      asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
          "%s:%s: cannot find SpeFormat element\n", driverName, functionName);
      return asynError;
  }
  dataFormatElement = xmlFindChildElement(speFormatElement, "DataFormat");
  if (!dataFormatElement) goto done;
  dataBlockElement = xmlFindChildElement(dataFormatElement, "DataBlock");
  if (!dataBlockElement) goto done;
  xmlSetProp(dataBlockElement, (const xmlChar*)"pixelFormat", (const xmlChar*)dataTypeString);
  sprintf(tempString, "%lu", (unsigned long)arrayInfo.totalBytes);
  xmlSetProp(dataBlockElement, (const xmlChar*)"size", (const xmlChar*)tempString);
  xmlSetProp(dataBlockElement, (const xmlChar*)"stride", (const xmlChar*)tempString);
  dataBlockElement2 = xmlFindChildElement(dataBlockElement, "DataBlock");
  if (!dataBlockElement2) goto done;
  xmlSetProp(dataBlockElement2, (const xmlChar*)"size", (const xmlChar*)tempString);
  sprintf(tempString, "%d", nx);
  xmlSetProp(dataBlockElement2, (const xmlChar*)"width", (const xmlChar*)tempString);
  sprintf(tempString, "%d", ny);
  xmlSetProp(dataBlockElement2, (const xmlChar*)"height", (const xmlChar*)tempString);

  // Set the required values in the Calibrations element
  calibrationsElement = xmlFindChildElement(speFormatElement, "Calibrations");
  if (!calibrationsElement) goto done;
  wavelengthMappingElement = xmlFindChildElement(calibrationsElement, "WavelengthMapping");
  if (!wavelengthMappingElement) goto done;
  wavelengthElement = xmlFindChildElement(wavelengthMappingElement, "Wavelength");
  if (!wavelengthElement) goto done;
  xmlNodeSetContent(wavelengthElement, (const xmlChar*)calibrationString);
  sensorInformationElement = xmlFindChildElement(calibrationsElement, "SensorInformation");
  if (!sensorInformationElement) goto done;
  sprintf(tempString, "%d", nx);
  xmlSetProp(sensorInformationElement, (const xmlChar*)"width", (const xmlChar*)tempString);
  sprintf(tempString, "%d", ny);
  xmlSetProp(sensorInformationElement, (const xmlChar*)"height", (const xmlChar*)tempString);
  sensorMappingElement = xmlFindChildElement(calibrationsElement, "SensorMapping");
  sprintf(tempString, "%d", nx);
  xmlSetProp(sensorMappingElement, (const xmlChar*)"width", (const xmlChar*)tempString);
  sprintf(tempString, "%d", ny);
  xmlSetProp(sensorMappingElement, (const xmlChar*)"height", (const xmlChar*)tempString);
  xmlError = false;
  
done:
  if (xmlError) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, 
      "%s::%s XML parsing error\n", driverName, functionName);
  }  
  else {
    int nChars = xmlDocDump(fp, mSPEDoc);
    if (nChars < 0) {
      asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, 
        "%s::%s error calling xmlDocDump\n", driverName, functionName);
    }
    else {
      asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, 
        "%s::%s xmlDocDump wrote %d bytes\n", driverName, functionName, nChars);
    }
  }
  // Close the file
  fclose(fp);
  free(calibration);
  free(calibrationString);
  
  return DRV_SUCCESS;
}


// C utility functions to tie in with EPICS

static void andorStatusTaskC(void *drvPvt)
{
  AndorCCD *pPvt = (AndorCCD *)drvPvt;

  pPvt->statusTask();
}


static void andorDataTaskC(void *drvPvt)
{
  AndorCCD *pPvt = (AndorCCD *)drvPvt;

  pPvt->dataTask();
}

/** IOC shell configuration command for Andor driver
  * \param[in] portName The name of the asyn port driver to be created.
  * \param[in] installPath The path to the Andor directory containing the detector INI files, etc.
  *            This can be specified as an empty string ("") for new detectors that don't use the INI
  * \param[in] cameraSerial The serial number of the desired camera.
  * \param[in] shamrockID The index number of the Shamrock spectrograph, if installed.
  *            0 is the first Shamrock in the system.  Ignored if there are no Shamrocks.  
  * \param[in] maxBuffers The maximum number of NDArray buffers that the NDArrayPool for this driver is 
  *            allowed to allocate. Set this to -1 to allow an unlimited number of buffers.
  * \param[in] maxMemory The maximum amount of memory that the NDArrayPool for this driver is 
  *            allowed to allocate. Set this to -1 to allow an unlimited amount of memory.
  *            files on Windows, but must be a valid path on Linux.
  * \param[in] priority The thread priority for the asyn port driver thread
  * \param[in] stackSize The stack size for the asyn port driver thread
  */
extern "C" {
int andorCCDConfig(const char *portName, const char *installPath, int cameraSerial, int shamrockID,
                   int maxBuffers, size_t maxMemory, int priority, int stackSize)
{
  /*Instantiate class.*/
  new AndorCCD(portName, installPath, cameraSerial, shamrockID, maxBuffers, maxMemory, priority, stackSize);
  return(asynSuccess);
}


/* Code for iocsh registration */

/* andorCCDConfig */
static const iocshArg andorCCDConfigArg0 = {"Port name", iocshArgString};
static const iocshArg andorCCDConfigArg1 = {"installPath", iocshArgString};
static const iocshArg andorCCDConfigArg2 = {"cameraSerial", iocshArgInt};
static const iocshArg andorCCDConfigArg3 = {"shamrockID", iocshArgInt};
static const iocshArg andorCCDConfigArg4 = {"maxBuffers", iocshArgInt};
static const iocshArg andorCCDConfigArg5 = {"maxMemory", iocshArgInt};
static const iocshArg andorCCDConfigArg6 = {"priority", iocshArgInt};
static const iocshArg andorCCDConfigArg7 = {"stackSize", iocshArgInt};
static const iocshArg * const andorCCDConfigArgs[] =  {&andorCCDConfigArg0,
                                                       &andorCCDConfigArg1,
                                                       &andorCCDConfigArg2,
                                                       &andorCCDConfigArg3,
                                                       &andorCCDConfigArg4,
                                                       &andorCCDConfigArg5,
                                                       &andorCCDConfigArg6,
                                                       &andorCCDConfigArg7};

static const iocshFuncDef configAndorCCD = {"andorCCDConfig", 8, andorCCDConfigArgs};
static void configAndorCCDCallFunc(const iocshArgBuf *args)
{
    andorCCDConfig(args[0].sval, args[1].sval, args[2].ival, args[3].ival, 
                   args[4].ival, args[5].ival, args[6].ival, args[7].ival);
}

static void andorCCDRegister(void)
{

    iocshRegister(&configAndorCCD, configAndorCCDCallFunc);
}

epicsExportRegistrar(andorCCDRegister);
}
