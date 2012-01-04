/**
 * Area Detector driver for the Andor CCD.
 *
 * @author Matthew Pearson
 * @date June 2009
 *
 * Updated Dec 2011 for Asyn 4-17 and areaDetector 1-7 
 *
 */

#include <iostream>
#include <fstream>

#include "AndorCCD.h"
#include "atmcdLXd.h"

using std::cout;
using std::endl;
using std::flush;
using std::ofstream;

//Definitions of static class data members

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

const epicsInt32 AndorCCD::AShutterAuto = 0;
const epicsInt32 AndorCCD::AShutterOpen = 1;
const epicsInt32 AndorCCD::AShutterClose = 2;

const epicsInt32 AndorCCD::AFFTIFF = 0;
const epicsInt32 AndorCCD::AFFBMP = 1;
const epicsInt32 AndorCCD::AFFSIF = 2;
const epicsInt32 AndorCCD::AFFEDF = 3;
const epicsInt32 AndorCCD::AFFRAW = 4;
const epicsInt32 AndorCCD::AFFTEXT = 5;

const std::string AndorCCD::INSTALL_PATH = "/usr/local/etc/andor";

//C Function prototypes to tie in with EPICS
static void andorStatusTaskC(void *drvPvt);
static void andorDataTaskC(void *drvPvt);

/**
 * Constructor for AndorCCD::AndorCCD.
 *
 */
AndorCCD::AndorCCD(const char *portName, int maxBuffers, size_t maxMemory, int maxSizeX, int maxSizeY)

  : ADDriver(portName, 1, NUM_ANDOR_DET_PARAMS, maxBuffers, maxMemory, 0, 0,
	     ASYN_CANBLOCK, 1, 0, 0)
{

  int status = asynSuccess;
  int param1 = 0;
  const char *functionName = "AndorCCD::AndorCCD";

  cout << "Constructing AndorCCD driver..." << endl;

  /* Set some default values for parameters */
  this->lock();

  createParam(AndorCoolerParamString,             asynParamInt32, &AndorCoolerParam);
  createParam(AndorShutdownParamString,           asynParamInt32, &AndorShutdownParam);
  createParam(AndorStartupParamString,            asynParamInt32, &AndorStartupParam);
  createParam(AndorImageModeAMultiParamString,    asynParamInt32, &AndorImageModeAMultiParam);
  createParam(AndorACTInKineticsParamString,      asynParamInt32, &AndorACTInKineticsParam);
  createParam(AndorANumInKineticsParamString,     asynParamInt32, &AndorANumInKineticsParam);
  createParam(AndorFKHeightParamString,           asynParamInt32, &AndorFKHeightParam);
  createParam(AndorFKHBinningParamString,         asynParamInt32, &AndorFKHBinningParam);
  createParam(AndorFKVBinningParamString,         asynParamInt32, &AndorFKVBinningParam);
  createParam(AndorFKOffsetParamString,           asynParamInt32, &AndorFKOffsetParam);
  createParam(AndorTempStatusMessageString,       asynParamOctet, &AndorTempStatusMessage);
  createParam(AndorMessageString,                 asynParamOctet, &AndorMessage);
  createParam(AndorShutterModeString,             asynParamInt32, &AndorShutterMode);
  createParam(AndorShutterExTTLString,            asynParamInt32, &AndorShutterExTTL);
  createParam(AndorFileFormatString,              asynParamInt32, &AndorFileFormat);
  createParam(AndorPalFileNameString,             asynParamOctet, &AndorPalFileName);
  createParam(AndorAdcSpeedString,                asynParamInt32, &AndorAdcSpeed);

  this->unlock();

  //Create the epicsEvent for signaling to the status task when parameters should have changed.
  //This will cause it to do a poll immediately, rather than wait for the poll time period.
  this->statusEvent = epicsEventMustCreate(epicsEventEmpty);
  if (!this->statusEvent) {
    printf("%s:%s epicsEventCreate failure for start event\n", driverName, functionName);
    return;
  }

  //Use this to signal the data acquisition task that acquisition has started.
  this->dataEvent = epicsEventMustCreate(epicsEventEmpty);
  if (!this->dataEvent) {
    printf("%s:%s epicsEventCreate failure for data event\n", driverName, functionName);
    return;
  }

  //The camera is not initialized yet.
  mRunning = 0;

  //Set default driver start parameters for binning and sub-area readout.
  //Use 1 based (which Andor uses. areaDetector is 0 based)
  mXBinning = 1;
  mYBinning = 1;
  mXStart = 1;
  mYStart = 1;
  mXEnd = maxSizeX;
  mYEnd = maxSizeY;
  mXSize = maxSizeX;
  mYSize = maxSizeY;  
  mXMaxSize = maxSizeX;
  mYMaxSize = maxSizeY;

  mShutterExTTL = 1; //Use high TTL signal for external shutter
  mShutterMode = AShutterAuto;  //Auto mode
  mShutterCloseTime = 0; //milliseconds
  mShutterOpenTime = 0; //milliseconds
  
  //Initialize camera
  initializeCCD(INSTALL_PATH);

  try {
    checkStatus(GetAvailableCameras(reinterpret_cast<long *>(&param1)));
    cout << "Number of cameras found: " << param1 << endl;
  } catch (const std::string &e) {
    cout << e << endl;
  }

  /* Set some default values for parameters */
  this->lock();

  status =  setStringParam(ADManufacturer, "Andor");
  status |= setStringParam(ADModel, "CCD");
  status |= setIntegerParam(ADSizeX, maxSizeX-1);
  status |= setIntegerParam(ADSizeY, maxSizeY-1);
  status |= setIntegerParam(ADMaxSizeX, maxSizeX-1);
  status |= setIntegerParam(ADMaxSizeY, maxSizeY-1);  
  status |= setIntegerParam(ADImageMode, ADImageSingle);
  status |= setIntegerParam(ADTriggerMode, AndorCCD::ATInternal);
  status |= setDoubleParam (ADAcquireTime, 1.0);
  status |= setDoubleParam (ADAcquirePeriod, 1.0);
  status |= setIntegerParam(ADNumImages, 1);
  status |= setIntegerParam(AndorFKHeightParam, 1);
  status |= setIntegerParam(AndorFKHBinningParam, 1);
  status |= setIntegerParam(AndorFKVBinningParam, 1);
  status |= setIntegerParam(AndorFKOffsetParam, 0);
  status |= setIntegerParam(NDArraySizeX, mXSize);
  status |= setIntegerParam(NDArraySizeY, mYSize);
  status |= setIntegerParam(NDArraySize, mXSize*mYSize*2);
  
  callParamCallbacks();
  this->unlock();

  AAModeCurrent = AASingle;
  AAModeMulti = AAAccumulate;
  mACTInKinetics = 0;
  mTriggerMode = ATInternal;
  mFileFormat = AFFTIFF;
  
  
  if (status) {
    printf("%s: unable to set camera parameters\n", functionName);
    return;
  }

  //Define the polling periods for the status thread.
  mPollingPeriod = 0.2; //seconds
  mFastPollingPeriod = 0.05; //seconds

  mAcquiringData = 0;

  //Allocate space for data (single image)
  mDataSize = mXSize * mYSize;
  mData = NULL;
  mData = (long *) calloc(mDataSize, sizeof(long));
  if (mData == NULL) {
    cout << driverName << ":" << functionName << "  ERROR: Could not allocate enough memory for data" << endl;
    return;
  }
  

  /* Create the thread that updates the detector status */
  status = (epicsThreadCreate("AndorStatusTask",
			      epicsThreadPriorityMedium,
			      epicsThreadGetStackSize(epicsThreadStackMedium),
			      (EPICSTHREADFUNC)andorStatusTaskC,
			      this) == NULL);
  if (status) {
    printf("%s:%s epicsThreadCreate failure for status task\n",
	   driverName, functionName);
    return;
  }

  
  /* Create the thread that does data readout */
  status = (epicsThreadCreate("AndorDataTask",
			      epicsThreadPriorityMedium,
			      epicsThreadGetStackSize(epicsThreadStackMedium),
			      (EPICSTHREADFUNC)andorDataTaskC,
			      this) == NULL);
  if (status) {
    printf("%s:%s epicsThreadCreate failure for data task\n",
	   driverName, functionName);
    return;
  }


  

  

}

AndorCCD::~AndorCCD() 
{
  //NOTE this destructor doesn't seem to be called.
  printf("Destructing.\n");
}



/**
 * Initialize the CCD camera using the built SDK Initialize(path) function. 
 * @param path The path to the local install of the SDK config files (usually /usr/local/etc/andor)
 */
void AndorCCD::initializeCCD(const std::string &path)
{
  //Initialize camera
  try {
    cout << "Initializing CCD...\n" << endl;
    checkStatus(Initialize(const_cast<char *>(path.c_str())));
    this->lock();
    setStringParam(AndorMessage, "Camera successfully initialized.");
    runAtInitialization();
    mRunning = 1;
    callParamCallbacks();
    this->unlock();
  } catch (const std::string &e) {
    cout << e << endl;
  }
}


/**
 * Shutdown the CCD.
 */
void AndorCCD::shutdownCCD(void)
{
  try {
    cout << "Shutdown and freeing up memory..." << endl;
    this->lock();
    checkStatus(FreeInternalMemory());
    checkStatus(ShutDown());
    setStringParam(AndorMessage, "Camera successfully shutdown. Restart IOC.");
    //Free data memory
    free(mData);
    mRunning = 0;
    callParamCallbacks();
    this->unlock();
  } catch (const std::string &e) {
    cout << e << endl;
  }
}


/**
 * Run this after the SDK driver has been initilaized. 
 * It reads some detector information and prints to stdout.
 */
asynStatus AndorCCD::runAtInitialization(void)
{
  int param1 = 0;
  int xsize = 0;
  int ysize = 0;
  unsigned int uIntParam1 = 0;
  unsigned int uIntParam2 = 0;
  unsigned int uIntParam3 = 0;
  unsigned int uIntParam4 = 0;
  unsigned int uIntParam5 = 0;
  unsigned int uIntParam6 = 0;

  cout << "Andor CCD camera information:" << endl;
  try {
    checkStatus(GetCameraSerialNumber(&param1));
    cout << "  serial number: " << param1 << endl; 
    checkStatus(GetHardwareVersion(&uIntParam1,&uIntParam2,&uIntParam3,&uIntParam4,&uIntParam5,&uIntParam6));
    cout << "  PCB Version: " << uIntParam1 << endl;
    cout << "  Flex File Version: " << uIntParam2 << endl;
    cout << "  Firmware Version: " << uIntParam5 << endl;
    cout << "  Firmware Build: " << uIntParam6 << endl;
    checkStatus(GetDetector(&xsize, &ysize));
    cout << "  xpixels: " << xsize << endl;
    cout << "  ypixels: " << ysize << endl;
    checkStatus(GetNumberAmp(&param1));
    cout << "  Number of amplifier channels: " << param1 << endl;
    checkStatus(GetNumberADChannels(&param1));
    cout << "  Number of ADC channels: " << param1 << endl;
    //Code to find the ADC with the fastest sampling rate.
    //This is commented out because the ADC channel can be set by the database now.
    /*float maxSpeed = 0.0;
    int fastestChannel = 0;
    for (int i=0; i<param1; i++) {
      float temp = 0.0;
      checkStatus(GetHSSpeed(i,0,0,&temp));
      if (temp > maxSpeed) {
	maxSpeed = temp;
	fastestChannel = i;
      }
    }
    cout << "  Defaulting to the fastest ADC channel, channel: " << fastestChannel << " (" << maxSpeed << ")." << endl;
    SetADChannel(fastestChannel);
    SetHSSpeed(0, 0);
    setIntegerParam(AndorAdcSpeed, fastestChannel);*/
    
  } catch (const std::string &e) {
    cout << e << endl;
    return(asynError);
  }

  //Set some detector size parameters in param lib 
  //so that they can read by user 
  //(areaDetector is 0 based, so I will set 0->(size-1))
  mXEnd = xsize;
  mYEnd = ysize;
  mXSize = xsize;
  mYSize = ysize;
  mXMaxSize = xsize - 1;
  mYMaxSize = ysize - 1;
    
  setIntegerParam(ADSizeX, mXEnd-1);
  setIntegerParam(ADSizeY, mYEnd-1);
  setIntegerParam(ADMaxSizeX, mXEnd-1);
  setIntegerParam(ADMaxSizeY, mYEnd-1);
  setIntegerParam(ADMinX, mXStart-1);
  setIntegerParam(ADMinY, mYStart-1);
  setIntegerParam(ADBinX, mXBinning);
  setIntegerParam(ADBinY, mYBinning);
  
  
  cout << "Setting read mode to be Image, and readout to be full image." << endl;
  try {
    checkStatus(SetReadMode(ARImage));
    cout << "SetImage("<<mXBinning<<","<<mYBinning<<","<<mXStart<<","<<mXEnd<<","<<mYStart<<","<<mYEnd<<");"<<endl;
    checkStatus(SetImage(mXBinning, mYBinning, mXStart, mXEnd, mYStart, mYEnd));
  } catch (const std::string &e) {
    cout << e << endl;
    return(asynError);
  }

  cout << "Setting shutter control parameters:" << endl;
  try {
    checkStatus(SetShutter(mShutterExTTL, mShutterMode, mShutterCloseTime, mShutterOpenTime));
    cout << "SetShutter("<<mShutterExTTL<<", "<<mShutterMode<<", "<<mShutterCloseTime<<", "<<mShutterOpenTime<<")"<<endl;
  } catch (const std::string &e) {
    cout << e << endl;
    return(asynError);
  }

  
  /* Send a signal to the poller task which will make it do a poll, and switch to the fast poll rate */
  epicsEventSignal(statusEvent);

  return asynSuccess;

}


asynStatus AndorCCD::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    int function = pasynUser->reason;
    int adstatus = 0;
    int FKheight = 0;
    int FKseries = 0;
    double FKtime = 0;
    int FKmode = 4;
    int FKhbin = 0;
    int FKvbin = 0;
    int FKoffset = 0;

    asynStatus status = asynSuccess;
    const char *functionName = "AndorCCD::writeInt32";

    if (function == ADAcquire) {
      getIntegerParam(ADStatus, &adstatus);
      if (value && (adstatus == ADStatusIdle)) {
	try {
	  mAcquiringData = 1;
	  //We send an event at the bottom of this function.
	  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, Acquiring data...\n", functionName);
	} catch (const std::string &e) {
	  cout << e << endl;
	  return(asynError);
	}
      }
      if (!value && (adstatus != ADStatusIdle)) {
	try {
	  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, AbortAcquisition()\n", functionName);
	  checkStatus(AbortAcquisition());
	  mAcquiringData = 0;
	  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, FreeInternalMemory()\n", functionName);
	  checkStatus(FreeInternalMemory());
	  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, CancelWait()\n", functionName);
	  checkStatus(CancelWait());
	} catch (const std::string &e) {
	  cout << e << endl;
	  return(asynError);
	} 
      }
    }
    else if (function == ADTriggerMode) {
      try {
	if (value == 0) {
	  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, SetTriggerMode(%d)\n", functionName, ATInternal);
	  checkStatus(SetTriggerMode(ATInternal));
	  mTriggerMode = ATInternal;
	} else if (value == 1) {
	  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, SetTriggerMode(%d)\n", functionName, ATExternal);
	  checkStatus(SetTriggerMode(ATExternal));
	  mTriggerMode = ATExternal;
	}
      } catch (const std::string &e) {
	cout << e << endl;
	return(asynError);
      }
    }
    else if (function == AndorFileFormat) {
      try {
	if (value == 0) {
	  //TIFF file format
	  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, Setting TIFF file format.\n", functionName);
	  mFileFormat = AFFTIFF;
	} else if (value == 1) {
	  //BMP file format
	  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, Setting BMP file format.\n", functionName);
	  mFileFormat = AFFBMP;
	} else if (value == 2) {
	  //SIF file format
	  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, Setting SIF file format.\n", functionName);
	  mFileFormat = AFFSIF;
	} else if (value == 3) {
	  //EDF file format
	  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, Setting EDF file format.\n", functionName);
	  mFileFormat = AFFEDF;
	} else if (value == 4) {
	  //RAW file format
	  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, Setting RAW file format.\n", functionName);
	  mFileFormat = AFFRAW;
	} else if (value == 5) {
	  //TEXT file format
	  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, Setting TEXT file format.\n", functionName);
	  mFileFormat = AFFTEXT;
	} else {
	  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, Unknown file format.\n", functionName);
	  return asynError;
	}
      } catch (const std::string &e) {
	cout << e << endl;
	return(asynError);
      }
    }
    else if (function == ADImageMode) {
      try {
	if (value == 0) {
	  //Single image mode
	  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, Setting single image mode.\n", functionName);
	  checkStatus(SetAcquisitionMode(AASingle));
	  setIntegerParam(ADNumImages, 1);
	  AAModeCurrent = AASingle;
	} else if (value == 1) {
	  //Multiple image mode (by default do accumulate mode. Use AndorImageModeAMultiParam to change to other types.)
	  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, Setting multiple image (accumulate) mode.\n", functionName);
	  checkStatus(SetAcquisitionMode(AAAccumulate));
	  AAModeCurrent = AAAccumulate;
	} else if (value == 2) {
	  //Continues mode
	  //Set Run Till Abort mode here.
	  setStringParam(AndorMessage, "Continues mode not supported.");
	  callParamCallbacks();
	  return asynError;
	}
      } catch (const std::string &e) {
	cout << e << endl;
	return(asynError);
      }
    }
    else if (function == AndorImageModeAMultiParam) {
      try {
	//If we are not already in Multiple Image mode, do nothing here.
	if (AAModeCurrent != AASingle) {
	  if (value == 0) {
	    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, Setting multiple image (accumulate) mode.\n", functionName);
	    checkStatus(SetAcquisitionMode(AAAccumulate));
	    AAModeCurrent = AAAccumulate;
	  } else if (value == 1) {
	    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, Setting multiple image (kinetics) mode.\n", functionName);
	    checkStatus(SetAcquisitionMode(AAKinetics));
	    AAModeCurrent = AAKinetics;
	  } else if (value == 2) {
	    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, Setting multiple image (fast kinetics) mode.\n", functionName);
	    checkStatus(SetAcquisitionMode(AAFastKinetics));
	    //Also send the SetFastKineticsEx command here.
	    AAModeCurrent = AAFastKinetics;
	  }
	} else {
	  setStringParam(AndorMessage, "Not in multiple image mode. No action taken.");
	  callParamCallbacks();
	  return asynError;
	}
      } catch (const std::string &e) {
	cout << e << endl;
	return(asynError);
      }
    }
    else if (function == ADNumImages) {
      asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, Setting number of images....\n", functionName);
      try {
	if (AAModeCurrent == AAAccumulate) {
	  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, SetNumberAccumulations(%d).\n", functionName, value);
	  checkStatus(SetNumberAccumulations(value));
	} else if (AAModeCurrent == AAKinetics) {
	  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, SetNumberKinetics(%d).\n", functionName, value);
	  checkStatus(SetNumberKinetics(value));
	} else if (AAModeCurrent == AAFastKinetics) {
	  getIntegerParam(AndorFKHeightParam, &FKheight);
	  getIntegerParam(AndorFKHBinningParam, &FKhbin);
	  getIntegerParam(AndorFKVBinningParam, &FKvbin);
	  getIntegerParam(AndorFKOffsetParam, &FKoffset);
	  getDoubleParam(ADAcquireTime, &FKtime);
	  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, SetFastKineticsEx(%d,%d,%d,%d,%d,%d,%d).\n", functionName, FKheight, value, FKtime, FKmode, FKhbin, FKvbin, FKoffset);
	  checkStatus(SetFastKineticsEx(FKheight, value, FKtime, FKmode, FKhbin, FKvbin, FKoffset));
	} else {
	  //Force user to set this after defining the multiple acquisition mode
	  setStringParam(AndorMessage, "Not in multiple image mode. No action taken.");
	  callParamCallbacks();
	  return(asynError);
	}
      } catch (const std::string &e) {
	cout << e << endl;
	return(asynError);
      }
    }
    else if (function == ADNumExposures) {
    }
    else if (function == AndorCoolerParam) {
      try {
	if (value == 0) {
	  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, CoolerOFF().\n", functionName);
	  checkStatus(CoolerOFF());
	} else if (value == 1) {
	  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, CoolerON().\n", functionName);
	  checkStatus(CoolerON());
	}
      } catch (const std::string &e) {
	cout << e << endl;
	return(asynError);
      }
    }
    else if (function == AndorShutdownParam) {
      shutdownCCD();
    }
    else if (function == AndorStartupParam) {
      initializeCCD(INSTALL_PATH);
    }
    else if (function == AndorANumInKineticsParam) {
      try {
	if (AAModeCurrent == AAKinetics) {
	  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, Setting number of accumulations in kinetics mode...\n", functionName);
	  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, SetNumberAccumulations(%d).\n", functionName, value);
	  checkStatus(SetNumberAccumulations(value));
	} else {
	  setStringParam(AndorMessage, "Not in kinetics mode. No action taken.");
	  callParamCallbacks();
	  return(asynError);
	}
      } catch (const std::string &e) {
	cout << e << endl;
	return(asynError);
      }
    }
    else if (function == AndorFKHeightParam) {
      if (AAModeCurrent == AAFastKinetics) {
	asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, Setting FK Height Param...\n", functionName);
	try {
	  getIntegerParam(AndorFKHBinningParam, &FKhbin);
	  getIntegerParam(ADNumImages, &FKseries);
	  getIntegerParam(AndorFKVBinningParam, &FKvbin);
	  getIntegerParam(AndorFKOffsetParam, &FKoffset);
	  getDoubleParam(ADAcquireTime, &FKtime);
	  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, SetFastKineticsEx(%d,%d,%d,%d,%d,%d,%d).\n", functionName, value, FKseries, FKtime, FKmode, FKhbin, FKvbin, FKoffset);
	  checkStatus(SetFastKineticsEx(value, FKseries, FKtime, FKmode, FKhbin, FKvbin, FKoffset));
	} catch (const std::string &e) {
	  cout << e << endl;
	  return(asynError);
	}
      } else {
	setStringParam(AndorMessage, "Not in fast kinetics mode. No action taken.");
	callParamCallbacks();
	return(asynError);
      }
    }
    else if (function == AndorFKHBinningParam) {
      if (AAModeCurrent == AAFastKinetics) {
	asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, Setting FK Horizontal binning param...\n", functionName);
	try {
	  getIntegerParam(AndorFKHeightParam, &FKheight);
	  getIntegerParam(ADNumImages, &FKseries);
	  getIntegerParam(AndorFKVBinningParam, &FKvbin);
	  getIntegerParam(AndorFKOffsetParam, &FKoffset);
	  getDoubleParam(ADAcquireTime, &FKtime);
	  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, SetFastKineticsEx(%d,%d,%d,%d,%d,%d,%d).\n", functionName, FKheight, FKseries, FKtime, FKmode, value, FKvbin, FKoffset);
	  checkStatus(SetFastKineticsEx(FKheight, FKseries, FKtime, FKmode, value, FKvbin, FKoffset));
	} catch (const std::string &e) {
	  cout << e << endl;
	  return(asynError);
	}
      } else {
	setStringParam(AndorMessage, "Not in fast kinetics mode. No action taken.");
	callParamCallbacks();
	return(asynError);
      }
    }
    else if (function == AndorFKVBinningParam) {
      if (AAModeCurrent == AAFastKinetics) {
	asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, Setting FK Vertical binning param...\n", functionName);
	try {
	  getIntegerParam(AndorFKHeightParam, &FKheight);
	  getIntegerParam(ADNumImages, &FKseries);
	  getIntegerParam(AndorFKHBinningParam, &FKhbin);
	  getIntegerParam(AndorFKOffsetParam, &FKoffset);
	  getDoubleParam(ADAcquireTime, &FKtime);
	  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, SetFastKineticsEx(%d,%d,%d,%d,%d,%d,%d).\n", functionName, FKheight, FKseries, FKtime, FKmode, FKhbin, value, FKoffset);
	  checkStatus(SetFastKineticsEx(FKheight, FKseries, FKtime, FKmode, FKhbin, value, FKoffset));
	} catch (const std::string &e) {
	  cout << e << endl;
	  return(asynError);
	}
      } else {
	setStringParam(AndorMessage, "Not in fast kinetics mode. No action taken.");
	callParamCallbacks();
	return(asynError);
      }
    }
    else if (function == AndorFKOffsetParam) {
      if (AAModeCurrent == AAFastKinetics) {
	asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, Setting FK offset param...\n", functionName);
	try {
	  getIntegerParam(AndorFKHeightParam, &FKheight);
	  getIntegerParam(ADNumImages, &FKseries);
	  getIntegerParam(AndorFKVBinningParam, &FKvbin);
	  getIntegerParam(AndorFKHBinningParam, &FKhbin);
	  getDoubleParam(ADAcquireTime, &FKtime);
	  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, SetFastKineticsEx(%d,%d,%d,%d,%d,%d,%d).\n", functionName, FKheight, FKseries, FKtime, FKmode, FKhbin, FKvbin, value);
	  checkStatus(SetFastKineticsEx(FKheight, FKseries, FKtime, FKmode, FKhbin, FKvbin, value));
	} catch (const std::string &e) {
	  cout << e << endl;
	  return(asynError);
	}
      } else {
	setStringParam(AndorMessage, "Not in fast kinetics mode. No action taken.");
	callParamCallbacks();
	return(asynError);
      }
    }
    else if (function == ADBinX) {
      try {
	//If AreaDetector tries to set 0 for binning, assume it means 1.
	if (value==0) {
	  value++;
	}
	asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, Setting BinX...\n", functionName);
	asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, SetImage(%d,%d,%d,%d,%d,%d).\n", functionName, value, mYBinning, mXStart, mXEnd, mYStart, mYEnd);
	checkStatus(SetImage(static_cast<int>(value), mYBinning, mXStart, mXEnd, mYStart, mYEnd));
	mXBinning = static_cast<int>(value);
      } catch (const std::string &e) {
	cout << e << endl;
	return(asynError);
      }
    }
    else if (function == ADBinY) {
      try {
	//If AreaDetector tries to set 0 for binning, assume it means 1.
	if (value==0) {
	  value++;
	}
	asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, Setting BinY...\n", functionName);
	asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, SetImage(%d,%d,%d,%d,%d,%d).\n", functionName, mXBinning, value, mXStart, mXEnd, mYStart, mYEnd);
	checkStatus(SetImage(mXBinning, static_cast<int>(value), mXStart, mXEnd, mYStart, mYEnd));
	mYBinning = static_cast<int>(value);
      } catch (const std::string &e) {
	cout << e << endl;
	return(asynError);
      }
    }
    else if (function == ADMinX) {
      try {
	asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, Setting MinX...\n", functionName);
	asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, SetImage(%d,%d,%d,%d,%d,%d).\n", functionName, mXBinning, mYBinning, static_cast<int>(value)+1, mXEnd+(static_cast<int>(value)-mXStart)+1, mYStart, mYEnd);
	checkStatus(SetImage(mXBinning, mYBinning, static_cast<int>(value)+1, mXEnd+(static_cast<int>(value)-mXStart)+1, mYStart, mYEnd));
	mXEnd = mXEnd+(static_cast<int>(value)-mXStart)+1;
	mXStart = static_cast<int>(value)+1;	
      } catch (const std::string &e) {
	cout << e << endl;
	return(asynError);
      }
    }
    else if (function == ADMinY) {
      try {
	asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, Setting MinY...\n", functionName);
	asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, SetImage(%d,%d,%d,%d,%d,%d).\n", functionName, mXBinning, mYBinning, mXStart, mXEnd, static_cast<int>(value)+1, mYEnd+(static_cast<int>(value)-mYStart)+1);
	checkStatus(SetImage(mXBinning, mYBinning, mXStart, mXEnd, static_cast<int>(value)+1, mYEnd+(static_cast<int>(value)-mYStart)+1));
	mYEnd = mYEnd + (static_cast<int>(value)-mYStart)+1;
	mYStart = static_cast<int>(value)+1;
      } catch (const std::string &e) {
	cout << e << endl;
	return(asynError);
      }
    }
    else if (function == ADSizeX) {
      try {
	//If AreaDetector tries to set 0 for binning, assume it means 1.
	if (value==0) {
	  value++;
	}
	asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, Setting SizeX...\n", functionName);
	asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, SetImage(%d,%d,%d,%d,%d,%d).\n", functionName, mXBinning, mYBinning, mXStart, mXStart+static_cast<int>(value), mYStart, mYEnd);
	checkStatus(SetImage(mXBinning, mYBinning, mXStart, mXStart+static_cast<int>(value), mYStart, mYEnd));
	mXEnd = mXStart + static_cast<int>(value);
      } catch (const std::string &e) {
	cout << e << endl;
	return(asynError);
      }
    }
    else if (function == ADSizeY) {
      try {
	//If AreaDetector tries to set 0 for binning, assume it means 1.
	if (value==0) {
	  value++;
	}
	asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, Setting SizeY...\n", functionName);
	asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, SetImage(%d,%d,%d,%d,%d,%d).\n", functionName, mXBinning, mYBinning, mXStart, mXEnd, mYStart, mYStart+static_cast<int>(value));
	checkStatus(SetImage(mXBinning, mYBinning, mXStart, mXEnd, mYStart, mYStart+static_cast<int>(value)));
	mYEnd = mYStart + static_cast<int>(value);
      } catch (const std::string &e) {
	cout << e << endl;
	return(asynError);
      }
    }
    else if (function == ADShutterControl) {
      try {
	if (value == 0) { //Close shutter
	  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, SetShutter(%d,%d,%d,%d).\n", functionName, mShutterExTTL, AShutterClose, mShutterCloseTime, mShutterOpenTime);
	  checkStatus(SetShutter(mShutterExTTL, AShutterClose, mShutterCloseTime, mShutterOpenTime)); 
	  mShutterMode = AShutterClose;
	} else { //Open shutter (check current value of AndorShutterMode)
	  int aShutter = 999;
	  getIntegerParam(AndorShutterMode, &aShutter);
	  if (aShutter == AShutterAuto) {
	    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, SetShutter(%d,%d,%d,%d).\n", functionName, mShutterExTTL, AShutterAuto, mShutterCloseTime, mShutterOpenTime);
	    checkStatus(SetShutter(mShutterExTTL, AShutterAuto, mShutterCloseTime, mShutterOpenTime)); 
	    mShutterMode = AShutterAuto;
	  } else if (aShutter == AShutterOpen) {
	    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, SetShutter(%d,%d,%d,%d).\n", functionName, mShutterExTTL, AShutterOpen, mShutterCloseTime, mShutterOpenTime);
	    checkStatus(SetShutter(mShutterExTTL, AShutterOpen, mShutterCloseTime, mShutterOpenTime)); 
	    mShutterMode = AShutterOpen;
	  }
	}
      } catch (const std::string &e) {
	cout << e << endl;
	return(asynError);
      }
    }

    else if (function == AndorShutterMode) {
      try {
	checkStatus(SetShutter(mShutterExTTL, static_cast<int>(value), mShutterCloseTime, mShutterOpenTime)); 
	mShutterMode = static_cast<int>(value);
	asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, SetShutter(%d,%d,%d,%d).\n", functionName, mShutterExTTL, mShutterMode, mShutterCloseTime, mShutterOpenTime);
      } catch (const std::string &e) {
	cout << e << endl;
	return(asynError);
      }
    }

    else if (function == AndorShutterExTTL) {
      try {
	checkStatus(SetShutter(static_cast<int>(value), mShutterMode, mShutterCloseTime, mShutterOpenTime)); 
	mShutterExTTL = static_cast<int>(value);
	asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, SetShutter(%d,%d,%d,%d).\n", functionName, mShutterExTTL, mShutterMode, mShutterCloseTime, mShutterOpenTime);
      } catch (const std::string &e) {
	cout << e << endl;
	return(asynError);
      }
    }

    else if (function == AndorAdcSpeed) {
      try {
	checkStatus(SetADChannel(value));
	//Set fastest HS speed.
	checkStatus(SetHSSpeed(0, 0));
	asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, SetADCChannel(%d).\n", functionName, value);
	setIntegerParam(AndorAdcSpeed, value);
      } catch (const std::string &e) {
	cout << e << endl;
	return(asynError);
      }
    } else {
      status = ADDriver::writeInt32(pasynUser, value);
    }

    //Set in param lib so the user sees a readback straight away. We might overwrite this in the 
    //status task, depending on the parameter.
    status = setIntegerParam(function, value);

    //For a successful write, clear the error message.
    if (mRunning==1) {
      setStringParam(AndorMessage, " ");
    }

    /* Do callbacks so higher layers see any changes */
    callParamCallbacks();

    /* Send a signal to the poller task which will make it do a poll, and switch to the fast poll rate */
    epicsEventSignal(statusEvent);

    if (mAcquiringData) {
      //Also signal the data readout thread
      epicsEventSignal(dataEvent);
    }

    if (status)
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
              "%s:%s: error, status=%d function=%d, value=%d\n",
              driverName, functionName, status, function, value);
    else
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
              "%s:%s: function=%d, value=%d\n",
              driverName, functionName, function, value);
    return status;
}

asynStatus AndorCCD::readInt32(asynUser *pasynUser, epicsInt32 *value)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    const char *functionName = "AndorCCD::readInt32";

    int temp = 0;

    //Changing any of the following parameters requires recomputing the base image 
    if (function == AndorCoolerParam) {
      asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, Reading cooler status...\n", functionName);
      try {
	checkStatus(IsCoolerOn(&temp));
      } catch (const std::string &e) {
	cout << e << endl;
	return(asynError);
      }
    }
    else {
      status = ADDriver::readInt32(pasynUser, value);
    }

    *value = static_cast<epicsInt32>(temp);

    status = setIntegerParam(function, *value);
     
    callParamCallbacks();
    if (status)
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
              "%s:%s error, status=%d function=%d, value=%f\n",
              driverName, functionName, status, function, value);
    else
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
              "%s:%s: function=%d, value=%f\n",
              driverName, functionName, function, value);
  
    return status;
}




asynStatus AndorCCD::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    const char *functionName = "AndorCCD::writeFloat64";

    int minTemp = 0;
    int maxTemp = 0;

    int FKheight = 0;
    int FKseries = 0;
    int FKmode = 4;
    int FKhbin = 0;
    int FKvbin = 0;
    int FKoffset = 0;

    /* Changing any of the following parameters requires recomputing the base image */
    if (function == ADGain) {
    }
    else if (function == ADAcquireTime) {
      try {
	if (AAModeCurrent == AAFastKinetics) {
	  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, Setting exposure time for fast kinetics.\n", functionName);
	  getIntegerParam(AndorFKHeightParam, &FKheight);
	  getIntegerParam(AndorFKHBinningParam, &FKhbin);
	  getIntegerParam(ADNumImages, &FKseries);
	  getIntegerParam(AndorFKVBinningParam, &FKvbin);
	  getIntegerParam(AndorFKOffsetParam, &FKoffset);
	  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, SetFastKineticsEx(%d,%d,%f,%d,%d,%d,%d).\n", functionName, FKheight, FKseries, value, FKmode, FKhbin, FKvbin, FKoffset);
	  checkStatus(SetFastKineticsEx(FKheight, FKseries, value, FKmode, FKhbin, FKvbin, FKoffset));
	} else {
	  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, SetExposureTime(%f).\n", functionName, value);
	  checkStatus(SetExposureTime(value));
	}
      } catch (const std::string &e) {
	cout << e << endl;
	return(asynError);
      }
    }
    else if (function == ADAcquirePeriod) {
      try {
	if (AAModeCurrent == AAAccumulate) {
	  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, SetAccumulationCycleTime(%f).\n", functionName, value);
	  checkStatus(SetAccumulationCycleTime(value));
	} else if (AAModeCurrent == AAKinetics) {
	  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, SetKineticCycleTime(%f).\n", functionName, value);
	  checkStatus(SetKineticCycleTime(value));
	}
      } catch (const std::string &e) {
	cout << e << endl;
	return(asynError);
      }
    }
    else if (function == ADTemperature) {
      asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, Setting temperature value %f.\n", functionName, value);
      try {
	checkStatus(GetTemperatureRange(&minTemp, &maxTemp));
	asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, CCD Min Temp: %d, Max Temp %d.\n", functionName, minTemp, maxTemp);
	if ((static_cast<int>(value) > minTemp) & (static_cast<int>(value) < maxTemp)) {
	  checkStatus(SetTemperature(static_cast<int>(value)));
	} else {
	  setStringParam(AndorMessage, "Temperature is out of range.");
	  callParamCallbacks();
	  return(asynError);
	}
      } catch (const std::string &e) {
	cout << e << endl;
	return(asynError);
      }
    }
    else if (function == AndorACTInKineticsParam) {
      try {
	if (AAModeCurrent == AAKinetics) {
	  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, Setting accumulated cycle time in kinetics mode...\n", functionName);
	  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, SetAccumulationCycleTime(%f).\n", functionName, value);
	  checkStatus(SetAccumulationCycleTime(value));
	} else {
	  setStringParam(AndorMessage, "Not in kinetics mode. No action taken.");
	  callParamCallbacks();
	  return(asynError);
	}
      } catch (const std::string &e) {
	cout << e << endl;
	return(asynError);
      }
    }
    else if (function == ADShutterOpenDelay) {
      try {
	asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, Setting ADShutterOpenDelay to %f.\n", functionName, value);
	checkStatus(SetShutter(mShutterExTTL, mShutterMode, mShutterCloseTime, static_cast<int>(value))); 
	mShutterOpenTime = static_cast<int>(value);
	asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, SetShutter(%d,%d,%d,%d).\n", functionName, mShutterExTTL, mShutterMode, mShutterCloseTime, mShutterOpenTime);
      } catch (const std::string &e) {
	cout << e << endl;
	return(asynError);
      }
    }
    else if (function == ADShutterCloseDelay) {
      try {
	asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, Setting ADShutterCloseDelay to %f.\n", functionName, value);
	checkStatus(SetShutter(mShutterExTTL, mShutterMode, static_cast<int>(value), mShutterOpenTime)); 
	mShutterCloseTime = static_cast<int>(value);
	asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, SetShutter(%d,%d,%d,%d).\n", functionName, mShutterExTTL, mShutterMode, mShutterCloseTime, mShutterOpenTime);
      } catch (const std::string &e) {
	cout << e << endl;
	return(asynError);
      }
    }
    else {
      status = ADDriver::writeFloat64(pasynUser, value);
    }

    /* Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
     * status at the end, but that's OK */
    status = setDoubleParam(function, value);

    //For a successful write, clear the error message.
    setStringParam(AndorMessage, " ");

    /* Do callbacks so higher layers see any changes */
    callParamCallbacks();
    if (status)
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
              "%s:%s error, status=%d function=%d, value=%f\n",
              driverName, functionName, status, function, value);
    else
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
              "%s:%s: function=%d, value=%f\n",
              driverName, functionName, function, value);
    return status;
}

asynStatus AndorCCD::readFloat64(asynUser *pasynUser, epicsFloat64 *value)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    const char *functionName = "AndorCCD::readFloat64";

    int temp = 0;
    float exposure = 0.0;
    float accumulate = 0.0;
    float kinetic = 0.0;

    //Changing any of the following parameters requires recomputing the base image 
    if (function == ADGain) {
    }
    else if (function == ADAcquireTime) {
      asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, Reading exposure timimgs...\n", functionName);
      try {
	checkStatus(GetAcquisitionTimings(&exposure, &accumulate, &kinetic));
	asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, exposure is %f.\n", functionName, exposure);
	asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, accumulate is %f.\n", functionName, accumulate);
	asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, kinetic is %f.\n", functionName, kinetic);
	*value = exposure;
      } catch (const std::string &e) {
	cout << e << endl;
	return(asynError);
      }
    }
    else if (function == ADAcquirePeriod) {
      asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, Reading exposure timimgs...\n", functionName);
      try {
	checkStatus(GetAcquisitionTimings(&exposure, &accumulate, &kinetic));
	if (AAModeCurrent == AAAccumulate) {
	  *value = accumulate;
	} else if (AAModeCurrent == AAKinetics) {
	  *value = kinetic;
	}
	asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, exposure is %f.\n", functionName, exposure);
	asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, accumulate is %f.\n", functionName, accumulate);
	asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, kinetic is %f.\n", functionName, kinetic);
      } catch (const std::string &e) {
	  cout << e << endl;
	  return(asynError);
      }
    }
    else if (function == ADTemperature) {
      try {
	checkStatus(GetTemperature(&temp));
	*value = (epicsFloat64)temp;
      } catch (const std::string &e) {
	cout << e << endl;
	return(asynError);
      }
    }
    else {
      status = ADDriver::readFloat64(pasynUser, value);  
    }

    status = setDoubleParam(function, *value);

    callParamCallbacks();

    /* Send a signal to the poller task which will make it do a poll, and switch to the fast poll rate */
    epicsEventSignal(statusEvent);

    if (status)
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
              "%s:%s error, status=%d function=%d, value=%f\n",
              driverName, functionName, status, function, value);
    else
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
              "%s:%s: function=%d, value=%f\n",
              driverName, functionName, function, value);
  
    return status;
}





/**
 * Function to check the return status of Andor SDK library functions.
 * @param returnStatus The return status of the SDK function
 * @return 0=success. Does not return in case of failure.
 * @throw std::string An exception is thrown in case of failure.
 */
unsigned int AndorCCD::checkStatus(unsigned int returnStatus) throw(std::string)
{
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
    setStringParam(AndorTempStatusMessage, "Temperature has stabilized at set point.");
    return 0;
  } else if (returnStatus == DRV_TEMP_NOT_REACHED) {
    setStringParam(AndorTempStatusMessage, "Temperature has not reached setpoint.");
    return 0;
  } else if (returnStatus == DRV_TEMP_DRIFT) {
    setStringParam(AndorTempStatusMessage, "Temperature has stabilized but has since drifted.");
    return 0;
  } else if (returnStatus == DRV_TEMP_NOT_STABILIZED) {
    setStringParam(AndorTempStatusMessage, "Temperature has not stabilized.");
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
  } else {
    throw std::string("ERROR: Unknown error code returned from Andor SDK.");
  }

  return 0;
}


/**
 * Update status of detector. Meant to be run in own thread.
 */
void AndorCCD::statusTask(void)
{
  int value = 0;
  unsigned int uvalue = 0;
  unsigned int status = 0;
  float exposure = 0.0;
  float accumulate = 0.0;
  float kinetic = 0.0;
  
  float timeout = 0.0;

  unsigned int forcedFastPolls = 0;

  const char *functionName = "AndorCCD::statusTask";

  cout << "Status thread started..." << endl;

  while(1) {

    //Read timeout for polling freq.
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
      //cout << "Got status event" << endl;
      //We got an event, rather than a timeout.  This is because other software
      //knows that data has arrived, or device should have changed state (parameters changed, etc.).
      //Force a minimum number of fast polls, because the device status
      //might not have changed in the first few polls
      forcedFastPolls = 5;
    }

    this->lock();
    if (mRunning) {

      //cout << " Status poll." << endl;

      //Only read these if we are not acquiring data
      if (!mAcquiringData) {
      
	//Read cooler status
	try {
	  checkStatus(IsCoolerOn(&value));
	  status = setIntegerParam(AndorCoolerParam, value);
	} catch (const std::string &e) {
	  cout << e << endl;
	  setStringParam(AndorMessage, e.c_str());
	}
      
	//Read temperature of CCD
	try {
	  checkStatus(GetTemperature(&value));
	  status = setDoubleParam(ADTemperature, static_cast<double>(value));
	} catch (const std::string &e) {
	  cout << e << endl;
	  setStringParam(AndorMessage, e.c_str());
	}
      
	//Read acquisition timings, and set ADAcquireTime and ADAcquirePeriod (depending on mode).
	try {
	  if (AAModeCurrent == AAFastKinetics) {
	    checkStatus(GetFKExposureTime(&exposure));
	    status |= setDoubleParam(ADAcquireTime, exposure);
	  } else {
	    checkStatus(GetAcquisitionTimings(&exposure, &accumulate, &kinetic));
	    if (AAModeCurrent == AASingle) {
	      status |= setDoubleParam(ADAcquireTime, exposure);
	    } else if (AAModeCurrent == AAAccumulate) {
	      status |= setDoubleParam(ADAcquirePeriod, accumulate);
	    } else if (AAModeCurrent == AAKinetics) {
	      status |= setDoubleParam(ADAcquirePeriod, kinetic);
	    }
	  }
	} catch (const std::string &e) {
	  cout << e << endl;
	  setStringParam(AndorMessage, e.c_str());
	}

      }

      //Read detector status (idle, acquiring, error, etc.)
      try {
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
	cout << e << endl;
	setStringParam(AndorMessage, e.c_str());
      }
            
      /* Call the callbacks to update any changes */
      callParamCallbacks();
    }

    this->unlock();
        
  } //End of loop

}




/**
 * Do data readout from the detector. Meant to be run in own thread.
 */
void AndorCCD::dataTask(void)
{
  epicsUInt32 status = 0;
  char *errorString = NULL;
  unsigned char dataValid = 0;
  //  char filePath[256] = {0};
  char palFilePath[256] = {0};
  //  char fileName[256] = {0};
  char fullFileName[256] = {0};
  //float fparam = 0.0;
  epicsInt32 numImages = 0;
  epicsUInt32 currentAcqMode = 0;
  
  //long *dP = NULL;

  const char *functionName = "AndorCCD::dataTask";

  cout << "Data thread started..." << endl;

  while(1) {
    
    errorString = NULL;

    //Wait for event from main thread to signal that data acquisition has started.
    status = epicsEventWait(dataEvent);
              
    //if (status == epicsEventWaitOK) {
    //  cout << "Got data event" << endl;
    //}

    this->lock();
    if (mRunning) {
      //Sanity check that main thread thinks we are acquiring data
      if (mAcquiringData) {
	dataValid = 1;
	//Read current acqusition setting
	currentAcqMode = AAModeCurrent;
	//Read the number of images set
	getIntegerParam(ADNumImages, &numImages);
      } else {
	asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, Data thread is running but main thread thinks we are not acquiring.\n", functionName);
      }
    }
    this->unlock();
    

    if (dataValid) {
      
      //Trigger and wait for data from SDK
      try {
	asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, StartAcquisition().\n", functionName);
	checkStatus(StartAcquisition());

	if (currentAcqMode == AASingle) {
	  //Single image mode. Wait for acquisiton then save data.
	  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, WaitForAcquisition().\n", functionName);
	  checkStatus(WaitForAcquisition());
	  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, WaitForAcquisition has returned.\n", functionName);
	  //Save data
	  this->saveDataFrame(fullFileName, palFilePath);
	} else if (currentAcqMode == AAAccumulate) {
	  //Wait for all images and then readout the single accumulated image from the SDK
	  for (int i=0; i<numImages; i++) {
	    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, accumulate image %d.\n", functionName, i);
	    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, WaitForAcquisition().\n", functionName);
	    checkStatus(WaitForAcquisition());
	  }
	  //Save data
	  this->saveDataFrame(fullFileName, palFilePath);
	} else if (currentAcqMode == AAKinetics) {
	  //Wait for each image acquisition and then read out one by one as they acquired. Save each one.
	  for (int i=0; i<numImages; i++) {
	    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, kinetic image %d.\n", functionName, i);
	    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, WaitForAcquisition().\n", functionName);
	    checkStatus(WaitForAcquisition());
	    //Save data
	    this->saveDataFrame(fullFileName, palFilePath);
	  }
	} else {
	  this->lock();
	  setStringParam(AndorMessage, "Error in data thread. Acquisition mode not recognised.");
	  callParamCallbacks();
	  this->unlock();
	}
      } catch (const std::string &e) {
	cout << e << endl;
	errorString = const_cast<char *>(e.c_str());
      }
      

      /////////////////////////////////////

      
      this->lock();
      //Now clear main thread flag
      mAcquiringData = 0;
      setIntegerParam(ADAcquire, 0);
      //setIntegerParam(ADStatus, 0); //Dont set this as the status thread sets it.

      /* Call the callbacks to update any changes */
      callParamCallbacks();
      this->unlock();
    }
        
  } //End of loop

}


/**
 * Save a data frame using the Andor SDK file writing functions.
 * Also has the option to save data as plain text.
 */
void AndorCCD::saveDataFrame(char *fullFileName, char *palFilePath) 
{

  long *dP = NULL;
  char *errorString = NULL;

  const char *functionName = "AndorCCD::saveDataFrame";

  //Update data
  this->lock();
      
  ////////////////////////////////////
  //Put data into waveforms, or save to file
  
  //Check we haven't cancelled data acquisition before trying to save file
  if (mAcquiringData) { 
    
    if (mFileFormat == AFFTIFF) {
      asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, Saving data in TIFF format.\n", functionName);
    } else if (mFileFormat == AFFBMP) {
      asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, Saving data in BMP format.\n", functionName);
    } else if (mFileFormat == AFFSIF) {
      asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, Saving data in SIF format.\n", functionName);
    } else if (mFileFormat == AFFEDF) {
      asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, Saving data in EDF format.\n", functionName);
    } else if (mFileFormat == AFFRAW) {
      asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, Saving data in RAW format.\n", functionName);
    } else if (mFileFormat == AFFTEXT) {
      asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, Saving data in TEXT format.\n", functionName);
    } 
    
    this->createFileName(255, fullFileName);
    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, file name is %s.\n", functionName, fullFileName);
    getStringParam(AndorPalFileName, 255, palFilePath);
    
    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, Saving data...\n", functionName);

    try {
      if (mFileFormat == AFFTIFF) {
	//checkStatus(SaveAsTiff(fullFileName, palFilePath, 1, 1)); //Didn't work
	checkStatus(SaveAsTiffEx(fullFileName, palFilePath, 1, 1, 1));
      } else if (mFileFormat == AFFBMP) {
	checkStatus(SaveAsBmp(fullFileName, palFilePath, 0, 0));
      } else if (mFileFormat == AFFSIF) {
	checkStatus(SaveAsSif(fullFileName));
      } else if (mFileFormat == AFFEDF) {
	checkStatus(SaveAsEDF(fullFileName, 0));
      } else if (mFileFormat == AFFRAW) {
	checkStatus(SaveAsRaw(fullFileName, 1));
      } else if (mFileFormat == AFFTEXT) {
	//Get data into buffer and dump to file
	checkStatus(GetMostRecentImage(mData, mDataSize));
	
	ofstream datafile;
	datafile.open(fullFileName);
	
	if (datafile.is_open()) {
	  datafile << fullFileName << endl;
	  
	  dP = mData;
	  for (int d=0; d<mDataSize; ++d) {
	    datafile << *dP << endl;
	    //cout << "mData[" << d << "]: " << *dP << endl;
	    dP++;
	  }
	  
	  datafile << flush;
	  datafile.close();
	} else {
	  setStringParam(AndorMessage, "ERROR: Could not open file.");
	}
	
      }
    } catch (const std::string &e) {
      cout << e << endl;
      errorString = const_cast<char *>(e.c_str());
    }

    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s, Saving data.. Done!\n", functionName);
    
  }

  if (errorString != NULL) {
    setStringParam(AndorMessage, errorString);
    setIntegerParam(ADStatus, ADStatusError);
  }

  this->unlock();
  
}


//C utility functions to tie in with EPICS

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

/**
 * Config function for IOC shell.
 *
 * @param portName The ASYN port.
 * @param maxBuffers The maximum number of data frame buffers for the ADDriver class.
 * @param maxMemory The maximum memory size allowed in the ADDriver class.
 */
extern "C" int andorCCDConfig(const char *portName, int maxBuffers, size_t maxMemory, int maxSizeX, int maxSizeY)
{
  /*Instantiate class.*/
  new AndorCCD(portName, maxBuffers, maxMemory, maxSizeX, maxSizeY);
  return(asynSuccess);
}




