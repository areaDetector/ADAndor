/**
 * Area Detector driver for the Andor CCD.
 *
 * @author Matthew Pearson
 * @date June 2009
 *
 * Updated Dec 2011 for Asyn 4-17 and areaDetector 1-7 
 *
 * Major updates to get callbacks working, etc. by Mark Rivers Feb. 2011
 */

#ifndef ANDORCCD_H
#define ANDORCCD_H

#include <stddef.h>
#include <stdlib.h>
#include <stdarg.h>
#include <math.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <ctype.h>
#include <fcntl.h>
#include <sys/stat.h>

#include <string>

#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsEvent.h>
#include <epicsMutex.h>
#include <epicsString.h>
#include <epicsStdio.h>
#include <cantProceed.h>

#include <asynOctetSyncIO.h>

#include "ADDriver.h"

#ifdef _WIN32
#include "ATMCD32D.h"
#else
#include "atmcdLXd.h"
#endif

#define AndorCoolerParamString             "ANDOR_COOLER"
#define AndorTempStatusMessageString       "ANDOR_TEMP_STAT"
#define AndorMessageString                 "ANDOR_MESSAGE"
#define AndorShutterModeString             "ANDOR_SHUTTER_MODE"
#define AndorShutterExTTLString            "ANDOR_SHUTTER_EXTTL"
#define AndorPalFileNameString             "ANDOR_PAL_FILE_PATH"
#define AndorAccumulatePeriodString        "ANDOR_ACCUMULATE_PERIOD"
#define AndorAdcSpeedString                "ANDOR_ADC_SPEED"


/**
 * Driver class for Andor CCD. This inherits from ADDriver class in areaDetector.
 *
 */
class AndorCCD : public ADDriver {
 public:
  AndorCCD(const char *portName, int maxBuffers, size_t maxMemory, 
           const char *installPath, int priority, int stackSize);
  virtual ~AndorCCD();

  /* These are the methods that we override from ADDriver */
  virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
  virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
  virtual void report(FILE *fp, int details);

  // Should be private, but are called from C so must be public
  void statusTask(void);
  void dataTask(void);
  void shutdownCCD(void);

 protected:
  int AndorCoolerParam;
  #define FIRST_ANDOR_PARAM AndorCoolerParam
  int AndorTempStatusMessage;
  int AndorMessage;
  int AndorShutterMode;
  int AndorShutterExTTL;
  int AndorPalFileName;
  int AndorAccumulatePeriod;
  int AndorAdcSpeed;
  #define LAST_ANDOR_PARAM AndorAdcSpeed

 private:

  unsigned int checkStatus(unsigned int returnStatus);
  asynStatus setupAcquisition();
  asynStatus setupShutter(int command);
  void saveDataFrame();
  /**
   * Additional image mode to those in ADImageMode_t
   */
   static const epicsInt32 AImageFastKinetics;

  /**
   * List of acquisiton modes.
   */
  static const epicsUInt32 AASingle;
  static const epicsUInt32 AAAccumulate;
  static const epicsUInt32 AAKinetics;
  static const epicsUInt32 AAFastKinetics;
  static const epicsUInt32 AARunTillAbort;
  static const epicsUInt32 AATimeDelayedInt;

  /**
   * List of trigger modes.
   */
  static const epicsUInt32 ATInternal;
  static const epicsUInt32 ATExternal;
  static const epicsUInt32 ATExternalStart;
  static const epicsUInt32 ATExternalExposure;
  static const epicsUInt32 ATExternalFVB;
  static const epicsUInt32 ATSoftware;

  /**
   * List of detector status states
   */
  static const epicsUInt32 ASIdle;
  static const epicsUInt32 ASTempCycle;
  static const epicsUInt32 ASAcquiring;
  static const epicsUInt32 ASAccumTimeNotMet;
  static const epicsUInt32 ASKineticTimeNotMet;
  static const epicsUInt32 ASErrorAck;
  static const epicsUInt32 ASAcqBuffer;
  static const epicsUInt32 ASSpoolError;

  /**
   * List of detector readout modes.
   */
  static const epicsInt32 ARFullVerticalBinning;
  static const epicsInt32 ARMultiTrack;
  static const epicsInt32 ARRandomTrack;
  static const epicsInt32 ARSingleTrack;
  static const epicsInt32 ARImage;

  /**
   * List of shutter modes
   */
  static const epicsInt32 AShutterAuto;
  static const epicsInt32 AShutterOpen;
  static const epicsInt32 AShutterClose;

  /**
   * List of file formats
   */
  static const epicsInt32 AFFTIFF;
  static const epicsInt32 AFFBMP;
  static const epicsInt32 AFFSIF;
  static const epicsInt32 AFFEDF;
  static const epicsInt32 AFFRAW;
  static const epicsInt32 AFFTEXT;

  epicsEventId statusEvent;
  epicsEventId dataEvent;
  double mPollingPeriod;
  double mFastPollingPeriod;
  unsigned int mAcquiringData;
  char *mInstallPath;

  //Shutter control parameters
  float mAcquireTime;
  float mAcquirePeriod;
  float mAccumulatePeriod;

};

#define NUM_ANDOR_DET_PARAMS (&LAST_ANDOR_PARAM - &FIRST_ANDOR_PARAM + 1)

static const char *driverName = "andorCCDDetector";

#endif //ANDORCCD_H

