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
 */

#ifndef ANDORCCD_H
#define ANDORCCD_H

#include <libxml/parser.h>
#include <CCDMultiTrack.h>

#include "ADDriver.h"
#include "SPEHeader.h"

#define MAX_ENUM_STRING_SIZE 26
#define MAX_ADC_SPEEDS 16
#define MAX_PREAMP_GAINS 16
#define MAX_VS_PERIODS 16

#define AndorCoolerParamString             "ANDOR_COOLER"
#define AndorTempStatusMessageString       "ANDOR_TEMP_STAT"
#define AndorMessageString                 "ANDOR_MESSAGE"
#define AndorShutterModeString             "ANDOR_SHUTTER_MODE"
#define AndorShutterExTTLString            "ANDOR_SHUTTER_EXTTL"
#define AndorPalFileNameString             "ANDOR_PAL_FILE_PATH"
#define AndorAccumulatePeriodString        "ANDOR_ACCUMULATE_PERIOD"
#define AndorPreAmpGainString              "ANDOR_PREAMP_GAIN"
#define AndorEmGainString                  "ANDOR_EM_GAIN"
#define AndorEmGainModeString              "ANDOR_EM_GAIN_MODE"
#define AndorEmGainAdvancedString          "ANDOR_EM_GAIN_ADVANCED"
#define AndorAdcSpeedString                "ANDOR_ADC_SPEED"
#define AndorBaselineClampString           "ANDOR_BASELINE_CLAMP"
#define AndorReadOutModeString             "ANDOR_READOUT_MODE"
#define AndorFrameTransferModeString       "ANDOR_FT_MODE"
#define AndorVerticalShiftPeriodString     "ANDOR_VS_PERIOD"
#define AndorVerticalShiftAmplitudeString  "ANDOR_VS_AMPLITUDE"

/**
 * Structure defining an ADC speed for the ADAndor driver.
 *
 */
typedef struct {
  int ADCIndex;
  int AmpIndex;
  int HSSpeedIndex;
  float HSSpeed;
  int BitDepth;
  char *EnumString;
  int EnumValue;
} AndorADCSpeed_t;

/**
 * Structure defining a pre-amp gain for the ADAndor driver.
 *
 */
typedef struct {
  float Gain;
  char *EnumString;
  int EnumValue;
} AndorPreAmpGain_t;

/*
 * Structure defining a Vertical Shift Period for the ADAndor driver.
 */
typedef struct {
  float Period;
  int Index;
  char *EnumString;
  int EnumValue;
} AndorVSPeriod_t;

/**
 * Driver for Andor CCD cameras using version 2 of their SDK; inherits from ADDriver class in ADCore.
 *
 */
class AndorCCD : public ADDriver {
 public:
  AndorCCD(const char *portName, const char *installPath, int cameraSerial, int shamrockID,
           int maxBuffers, size_t maxMemory, int priority, int stackSize);
  virtual ~AndorCCD();

  /* These are the methods that we override from ADDriver */
  virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
  virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
  virtual asynStatus writeInt32Array(asynUser *pasynUser, epicsInt32 *value, size_t nElements);
  virtual void report(FILE *fp, int details);
  virtual asynStatus readEnum(asynUser *pasynUser, char *strings[], int values[], int severities[],
                              size_t nElements, size_t *nIn);

  // Should be private, but are called from C so must be public
  void statusTask(void);
  void dataTask(void);

 protected:
  int AndorCoolerParam;
#define FIRST_ANDOR_PARAM AndorCoolerParam
  int AndorTempStatusMessage;
  int AndorMessage;
  int AndorShutterMode;
  int AndorShutterExTTL;
  int AndorPalFileName;
  int AndorAccumulatePeriod;
  int AndorPreAmpGain;
  int AndorEmGain;
  int AndorEmGainMode;
  int AndorEmGainAdvanced;
  int AndorAdcSpeed;
  int AndorBaselineClamp;
  int AndorReadOutMode;
  int AndorFrameTransferMode;
  int AndorVerticalShiftPeriod;
  int AndorVerticalShiftAmplitude;
#define LAST_ANDOR_PARAM AndorVerticalShiftAmplitude

 private:

  unsigned int checkStatus(unsigned int returnStatus);
  asynStatus setupAcquisition();
  asynStatus setupShutter(int command);
  void saveDataFrame(int frameNumber);
  void setupADCSpeeds();
  void setupTrackDefn(int minX, int sizeX, int binX);
  void setupPreAmpGains();
  void setupVerticalShiftPeriods();
  unsigned int SaveAsSPE(char *fullFileName);
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
  static const epicsInt32 AShutterFullyAuto;
  static const epicsInt32 AShutterAlwaysOpen;
  static const epicsInt32 AShutterAlwaysClosed;
  static const epicsInt32 AShutterOpenFVP;
  static const epicsInt32 AShutterOpenAny;

  /**
   * List of file formats
   */
  static const epicsInt32 AFFTIFF;
  static const epicsInt32 AFFBMP;
  static const epicsInt32 AFFSIF;
  static const epicsInt32 AFFEDF;
  static const epicsInt32 AFFRAW;
  static const epicsInt32 AFFFITS;
  static const epicsInt32 AFFSPE;

  epicsEventId statusEvent;
  epicsEventId dataEvent;
  double mPollingPeriod;
  double mFastPollingPeriod;
  unsigned int mAcquiringData;
  char *mInstallPath;
  bool mExiting;
  int mExited;

  /**
   * ADC speed parameters
   */
  int mNumAmps;
  int mNumADCs;
  int mNumADCSpeeds;
  AndorADCSpeed_t mADCSpeeds[MAX_ADC_SPEEDS];
  int mTotalPreAmpGains;
  int mNumPreAmpGains;
  AndorPreAmpGain_t mPreAmpGains[MAX_PREAMP_GAINS];

  // Vertical Shift Period parameters
  int mTotalVSPeriods;
  int mNumVSPeriods;
  int mVSIndex;
  float mVSPeriod;
  AndorVSPeriod_t mVSPeriods[MAX_VS_PERIODS];

  //Shutter control parameters
  float mAcquireTime;
  float mAcquirePeriod;
  float mAccumulatePeriod;
  int mMinShutterOpenTime;
  int mMinShutterCloseTime;

  // Shamrock spectrometer ID
  int mShamrockId;

  // AndorCapabilities structure
  AndorCapabilities mCapabilities;

  CCDMultiTrack mMultiTrack;

  // EM Gain parameters 
  int mEmGainRangeLow;
  int mEmGainRangeHigh;

  // SPE file header
  tagCSMAHEAD *mSPEHeader;
  xmlDocPtr mSPEDoc;

  // Camera init status
  bool mInitOK;
};

#endif //ANDORCCD_H
