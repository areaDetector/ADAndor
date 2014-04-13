// SPEHeader.h.
// Mark Rivers, April 12, 2014
// This file originally was CHeader.h from Princeton Instruments.
// Modified to be OS-independent by using EPICS types.
// CHeader.h : definition of WinX/32 (CSMA-style) file header
//
// Copyright (C) 1997-2000 Roper Scientific
// All rights reserved
/////////////////////////////////////////////////////////////////////////////
//   This revision:  $Revision: 20 $
//   Last check-in   $Date: 2/06/03 3:36p $
//   Last changes by $Author: Hgrannis $
//   Changes:
/*   $Log: /WinX32 V2.5/INCLUDE/cheader.h $
*  
*  20    2/06/03 3:36p Hgrannis
*  File header 2.2 additions.
*  
*  19    7/25/02 3:22p Hgrannis
*  Added some AvGain.
*  
*  18    7/14/00 3:39p Terry
*  Put back "noscan" and "scramble" for 2.4 compatability
*  
*  17    7/05/00 10:34a Terry
*  Expand some pulser comments
*  
*  16    6/23/00 2:28p Terry
*  Add SpecSlitPosUnits & SpecGrooves
*  
*  15    6/21/00 8:06a Terry
*  Add new controllers to "CSMA" list; Add more members to header struct
*  
*  14    6/01/00 2:48p Terry
*  Cleanup file header, add new stuff
*  
*  13    5/19/00 7:29a Terry
*  Add pulse file name & single width/delay to header
*  
*/
/////////////////////////////////////////////////////////////////////////////

#if !defined(SPEHEADER_H)
#define SPEHEADER_H  /* define shows that include is now done */

/*#include "bvstruct.h" */

/* NOTE: the following values should not change, otherwise the header will */
/* be bleeped up.                                                          */
#define HDRNAMEMAX 120     /* max char str length for file name            */
#define USERINFOMAX 1000   /* user information space.                      */
#define COMMENTMAX 80      /* User comment string max length (5 comments)  */
#define LABELMAX 16        /* Label string max length.                     */
#define FILEVERMAX 16      /* File version string max length.              */
#define DATEMAX 10         /* string length of file creation date string as ddmmmyyyy\0 */
#define ROIMAX 10          /* Max size of roi array of structures.         */
#define TIMEMAX 7          /* Max time store as hhmmss\0                   */

#define HEADER_VERSION  2.2

enum CSCTRLTYPE {
  C_UNKNOWN         =  -1,
  C_ST1000          =  0,
  C_ST120NEW        =  1,
  C_ST120OLD        =  2,
  C_ST130           =  3,
  C_ST121           =  4,
  C_ST138           =  5,
  C_DC131           =  6,
  C_ST133           =  7,
  C_ST135           =  8,
  C_VICCD           =  9,
  C_ST116           = 10,
  C_OMA3            = 11,
  C_OMA4            = 12,
  C_ST143           = 13,
  C_VICCDBOX        = 14,
  C_MICROMAX        = 15,
  C_SPECTROMAX      = 16,
  C_MICROVIEW       = 17,
  C_LOW_COST_SPEC   = 18,
  C_ST133_5MHZ      = 19,
  C_EMPTY_5MHZ      = 20,
  C_EPIX_CONTROLLER = 21,
  C_PVCAM           = 22,
  C_GENERIC         = 23
};

#pragma pack(1)

typedef struct tagROIinfo       // ROI information
{
  epicsUInt16  startx;                 // left x start value.
  epicsUInt16  endx;                   // right x value.
  epicsUInt16  groupx;                 // amount x is binned/grouped in hw.
  epicsUInt16  starty;                 // top y start value.
  epicsUInt16  endy;                   // bottom y value.
  epicsUInt16  groupy;                 // amount y is binned/grouped in hw.
} ROIinfo;                      // 12 Bytes Each

typedef struct tagCSMAHEAD {
#ifdef __cplusplus                // STARTING
  tagCSMAHEAD();                  // ADDRESS
#endif                            // (decimal)
  epicsInt16    ControllerVersion;      //    0  Hardware Version
  epicsInt16    LogicOutput;            //    2  Definition of Output BNC
  epicsUInt16   AmpHiCapLowNoise;       //    4  Amp Switching Mode
  epicsUInt16   xDimDet;                //    6  Detector x dimension of chip.
  epicsInt16    mode;                   //    8  timing mode
  epicsFloat32  exp_sec;                //   10  alternitive exposure, in sec.
  epicsInt16    VChipXdim;              //   14  Virtual Chip X dim
  epicsInt16    VChipYdim;              //   16  Virtual Chip Y dim
  epicsUInt16   yDimDet;                //   18  y dimension of CCD or detector.
  char          date[DATEMAX];          //   20  date
  epicsInt16    VirtualChipFlag;        //   30  On/Off
  char          Spare_1[2];             //   32
  epicsInt16    noscan;                 //   34  Old number of scans - should always be -1
  epicsFloat32  DetTemperature;         //   36  Detector Temperature Set
  epicsInt16    DetType;                //   40  CCD/DiodeArray type
  epicsUInt16   xdim;                   //   42  actual # of pixels on x axis
  epicsInt16    stdiode;                //   44  trigger diode
  epicsFloat32  DelayTime;              //   46  Used with Async Mode
  epicsUInt16   ShutterControl;         //   50  Normal, Disabled Open, Disabled Closed
  epicsInt16    AbsorbLive;             //   52  On/Off
  epicsUInt16   AbsorbMode;             //   54  Reference Strip or File
  epicsInt16    CanDoVirtualChipFlag;   //   56  T/F Cont/Chip able to do Virtual Chip
  epicsInt16    ThresholdMinLive;       //   58  On/Off
  epicsFloat32  ThresholdMinVal;        //   60  Threshold Minimum Value
  epicsInt16    ThresholdMaxLive;       //   64  On/Off
  epicsFloat32  ThresholdMaxVal;        //   66  Threshold Maximum Value
  epicsInt16    SpecAutoSpectroMode;    //   70  T/F Spectrograph Used
  epicsFloat32  SpecCenterWlNm;         //   72  Center Wavelength in Nm
  epicsInt16    SpecGlueFlag;           //   76  T/F File is Glued
  epicsFloat32  SpecGlueStartWlNm;      //   78  Starting Wavelength in Nm
  epicsFloat32  SpecGlueEndWlNm;        //   82  Starting Wavelength in Nm
  epicsFloat32  SpecGlueMinOvrlpNm;     //   86  Minimum Overlap in Nm
  epicsFloat32  SpecGlueFinalResNm;     //   90  Final Resolution in Nm
  epicsInt16    PulserType;             //   94  0=None, PG200=1, PTG=2, DG535=3
  epicsInt16    CustomChipFlag;         //   96  T/F Custom Chip Used
  epicsInt16    XPrePixels;             //   98  Pre Pixels in X direction
  epicsInt16    XPostPixels;            //  100  Post Pixels in X direction
  epicsInt16    YPrePixels;             //  102  Pre Pixels in Y direction 
  epicsInt16    YPostPixels;            //  104  Post Pixels in Y direction
  epicsInt16    asynen;                 //  106  asynchron enable flag  0 = off
  epicsInt16    datatype;               //  108  experiment datatype
                                        //       0 =   FLOATING POINT
                                        //       1 =   LONG INTEGER
                                        //       2 =   INTEGER
                                        //       3 =   UNSIGNED INTEGER
  epicsInt16    PulserMode;             //  110  Repetitive/Sequential
  epicsUInt16   PulserOnChipAccums;     //  112  Num PTG On-Chip Accums
  epicsUInt32   PulserRepeatExp;        //  114  Num Exp Repeats (Pulser SW Accum)
  epicsFloat32  PulseRepWidth;          //  118  Width Value for Repetitive pulse (usec)
  epicsFloat32  PulseRepDelay;          //  122  Width Value for Repetitive pulse (usec)
  epicsFloat32  PulseSeqStartWidth;     //  126  Start Width for Sequential pulse (usec)
  epicsFloat32  PulseSeqEndWidth;       //  130  End Width for Sequential pulse (usec)
  epicsFloat32  PulseSeqStartDelay;     //  134  Start Delay for Sequential pulse (usec)
  epicsFloat32  PulseSeqEndDelay;       //  138  End Delay for Sequential pulse (usec)
  epicsInt16    PulseSeqIncMode;        //  142  Increments: 1=Fixed, 2=Exponential
  epicsInt16    PImaxUsed;              //  144  PI-Max type controller flag
  epicsInt16    PImaxMode;              //  146  PI-Max mode
  epicsInt16    PImaxGain;              //  148  PI-Max Gain
  epicsInt16    BackGrndApplied;        //  150  1 if background subtraction done
  epicsInt16    PImax2nsBrdUsed;        //  152  T/F PI-Max 2ns Board Used
  epicsUInt16   minblk;                 //  154  min. # of strips per skips
  epicsUInt16   numminblk;              //  156  # of min-blocks before geo skps
  epicsInt16    SpecMirrorLocation[2];  //  158  Spectro Mirror Location, 0=Not Present
  epicsInt16    SpecSlitLocation[4];    //  162  Spectro Slit Location, 0=Not Present
  epicsInt16    CustomTimingFlag;       //  170  T/F Custom Timing Used
  char     ExperimentTimeLocal[TIMEMAX];//  172  Experiment Local Time as hhmmss\0
  char     ExperimentTimeUTC[TIMEMAX];  //  179  Experiment UTC Time as hhmmss\0
  epicsInt16    ExposUnits;             //  186  User Units for Exposure
  epicsUInt16   ADCoffset;              //  188  ADC offset
  epicsUInt16   ADCrate;                //  190  ADC rate
  epicsUInt16   ADCtype;                //  192  ADC type
  epicsUInt16   ADCresolution;          //  194  ADC resolution
  epicsUInt16   ADCbitAdjust;           //  196  ADC bit adjust
  epicsUInt16   gain;                   //  198  gain
  char          Comments[5][COMMENTMAX];//  200  File Comments
  epicsUInt16   geometric;              //  600  geometric ops: rotate 0x01,
                                        //       reverse 0x02, flip 0x04
  char          xlabel[LABELMAX];       //  602  intensity display string
  epicsUInt16   cleans;                 //  618  cleans
  epicsUInt16   NumSkpPerCln;           //  620  number of skips per clean.
  epicsInt16    SpecMirrorPos[2];       //  622  Spectrograph Mirror Positions
  epicsFloat32  SpecSlitPos[4];         //  626  Spectrograph Slit Positions
  epicsInt16    AutoCleansActive;       //  642  T/F
  epicsInt16    UseContCleansInst;      //  644  T/F
  epicsInt16    AbsorbStripNum;         //  646  Absorbance Strip Number
  epicsInt16    SpecSlitPosUnits;       //  648  Spectrograph Slit Position Units
  epicsFloat32  SpecGrooves;            //  650  Spectrograph Grating Grooves
  epicsInt16    srccmp;                 //  654  number of source comp. diodes
  epicsUInt16   ydim;                   //  656  y dimension of raw data.
  epicsInt16    scramble;               //  658  0=scrambled,1=unscrambled
  epicsInt16    ContinuousCleansFlag;   //  660  T/F Continuous Cleans Timing Option
  epicsInt16    ExternalTriggerFlag;    //  662  T/F External Trigger Timing Option
  epicsInt32    lnoscan;                //  664  Number of scans (Early WinX)
  epicsInt32    lavgexp;                //  668  Number of Accumulations
  epicsFloat32  ReadoutTime;            //  672  Experiment readout time
  epicsInt16    TriggeredModeFlag;      //  676  T/F Triggered Timing Option
  unsigned long long XML_Offset;        //  678  XML Offset
  char          Spare_2[2];             //  686  
  char          sw_version[FILEVERMAX]; //  688  Version of SW creating this file
  epicsInt16    type;                   //  704  0=1000,1=new120,2=old120,3=130,
                                        //       st121=4,st138=5,dc131(PentaMax)=6,
                                        //       st133(MicroMax)=7,st135(GPIB)=8,
                                        //       VICCD=9, ST116(GPIB)=10,
                                        //       OMA3(GPIB)=11,OMA4=12
  epicsInt16    flatFieldApplied;       //  706  1 if flat field was applied.
  char          Spare_3[16];            //  708  
  epicsInt16    kin_trig_mode;          //  724  Kinetics Trigger Mode
  char          dlabel[LABELMAX];       //  726  Data label.
  char          Spare_4[436];           //  742
  char       PulseFileName[HDRNAMEMAX]; //  1178  Name of Pulser File with
                                        //           Pulse Widths/Delays (for Z-Slice)
  char      AbsorbFileName[HDRNAMEMAX]; // 1298 Name of Absorbance File (if File Mode)
  epicsUInt32   NumExpRepeats;          // 1418  Number of Times experiment repeated
  epicsUInt32   NumExpAccums;           // 1422  Number of Time experiment accumulated
  epicsInt16    YT_Flag;                // 1426  Set to 1 if this file contains YT data
  epicsFloat32  clkspd_us;              // 1428  Vert Clock Speed in micro-sec
  epicsInt16    HWaccumFlag;            // 1432  set to 1 if accum done by Hardware.
  epicsInt16    StoreSync;              // 1434  set to 1 if store sync used.
  epicsInt16    BlemishApplied;         // 1436  set to 1 if blemish removal applied.
  epicsInt16    CosmicApplied;          // 1438  set to 1 if cosmic ray removal applied
  epicsInt16    CosmicType;             // 1440  if cosmic ray applied, this is type.
  epicsFloat32  CosmicThreshold;        // 1442  Threshold of cosmic ray removal.
  epicsInt32    NumFrames;              // 1446  number of frames in file.
  epicsFloat32  MaxIntensity;           // 1450  max intensity of data (future)
  epicsFloat32  MinIntensity;           // 1454  min intensity of data (future)
  char          ylabel[LABELMAX];       // 1458  y axis label.
  epicsUInt16   ShutterType;            // 1474  shutter type.
  epicsFloat32  shutterComp;            // 1476  shutter compensation time.
  epicsUInt16   readoutMode;            // 1480  readout mode, full,kinetics, etc
  epicsUInt16   WindowSize;             // 1482  window size for kinetics only.
  epicsUInt16   clkspd;                 // 1484  clock speed for kinetics & frame transfer.
  epicsUInt16   interface_type;         // 1486  computer interface
                                        //       (isa-taxi, pci, eisa, etc.)
  epicsInt16    NumROIsInExperiment;    // 1488  May be more than the 10 allowed in
                                        //       this header (if 0, assume 1)
  char          Spare_5[16];            // 1490
  epicsUInt16   controllerNum;          // 1506  if multiple controller system will
                                        //       have controller number data came from.
                                        //       this is a future item.
  epicsUInt16   SWmade;                 // 1508  Which software package created this file
  epicsInt16    NumROI;                 // 1510  number of ROIs used. if 0 assume 1.
  ROIinfo       ROIinfoblk[ROIMAX];     // 1512  ROI information
  char          FlatField[HDRNAMEMAX];  // 1632  Flat field file name.
  char          background[HDRNAMEMAX]; // 1752  background sub. file name.
  char          blemish[HDRNAMEMAX];    // 1872  blemish file name.
  epicsFloat32  file_header_ver;        // 1992  version of this file header
  char          YT_Info[1000];          // 1996-2996  Reserved for YT information
  epicsInt32    WinView_id;             // 2996  == 0x01234567L if in use by WinView
  struct {                              //  X     Y
    epicsFloat64 offset;                // 3000  3489  offset for absolut data scaling
    epicsFloat64 factor;                // 3008  3497  factor for absolut data scaling
    char         current_unit;          // 3016  3505  selected scaling unit
    char         CalibReserved1;        // 3017  3506  reserved
    char         display_string[40];    // 3018  3507  string used for drawing axes...
    char         CalibReserved2[40];    // 3058  3547  reserved
    char         calib_valid;           // 3098  3587  flag if calibration is valid
    char         input_unit;            // 3099  3588  current input units for "calib_value"
    char         polynounit;            // 3100  3589  linear UNIT and used in the "polynocoeff"
    char         polynoorder;           // 3101  3590  ORDER of calibration POLYNOM
    char         calib_count;           // 3102  3591  valid calibration data pairs
    epicsFloat64 pixel_position[10];    // 3103  3592  pixel pos. of calibration data
    epicsFloat64 calib_value[10];       // 3183  3672  calibration VALUE at above pos
    epicsFloat64 polynocoeff[6];        // 3263  3752  polynom COEFFICIENTS
    epicsFloat64 laser_position;        // 3311  3800  laser wavenumber for relativ WN
    char         CalibReserved3;        // 3319  3808  reserved
    epicsUInt8   leftover_flag;         // 3320  3809  not used.
    char         user_label[40];        // 3321  3810  Calibration label
    char         expansion[128];        // 3361  3850  Expansion area
  } xcalibration,                       // 3000 -> 3488  X axis calibration
    ycalibration;                       // 3489 -> 3977  Y axis calibration
  char           Istring[40];           // 3978  special intensity scaling string
  char           Spare_6[25];           // 4018  
  epicsUInt8     SpecType;              // 4043 spectrometer type (acton, spex, etc.)
  epicsUInt8     SpecModel;             // 4044 spectrometer model (type dependent)
  epicsUInt8     PulseBurstUsed;        // 4045 pulser burst mode on/off
  epicsUInt32    PulseBurstCount;       // 4046 pulser triggers per burst
  epicsFloat64   PulseBurstPeriod;      // 4050 pulser burst period (in usec)
  epicsUInt8     PulseBracketUsed;      // 4058 pulser bracket pulsing on/off
  epicsUInt8     PulseBracketType;      // 4059 pulser bracket pulsing type
  epicsFloat64   PulseTimeConstFast;    // 4060 pulser slow exponential time constant (in usec)
  epicsFloat64   PulseAmplitudeFast;    // 4068 pulser fast exponential amplitude constant
  epicsFloat64   PulseTimeConstSlow;    // 4076 pulser slow exponential time constant (in usec)
  epicsFloat64   PulseAmplitudeSlow;    // 4084 pulser slow exponential amplitude constant
  epicsInt16     AnalogGain;            // 4092 analog gain
  epicsInt16     AvGainUsed;            // 4094 avalanche gain was used
  epicsInt16     AvGain;                // 4096 avalanche gain value
  epicsInt16     lastvalue;             // 4098 Always the LAST value in the header
} CSMAHead;                             // 4100 Bytes Total Header Size


#pragma pack()

#endif // CHEADER_H
