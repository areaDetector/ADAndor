areaDetector Andor driver
=========================

:author: Matthew Pearson, Oak Ridge National Laboratory and
         Mark Rivers, University of Chicago

.. contents:: Contents

Introduction
------------

This is an :doc:`../index` driver for CCD detectors from
`Andor Technology <http://www.andor.com>`__ using Version 2 of the Andor
Software Development Kit (SDK). It has been tested on the Andor iKon and
DU401 CCD cameras with USB interface, but should work with other cameras
as well. The driver is supported on 32-bit and 64- bit Linux and 32-bit
and 64-bit Windows.

The driver currently provides access to most of the features of the
Andor cameras:

-  All Andor acquisition modes (Single Scan, Accumulate, Kinetics, Run
   Till Abort, and Fast Kinetics
-  Control of the exposure time, accumulate cycle time, and kinetic
   cycle time
-  Support for all of the Andor trigger modes
-  Support for all of the Andor shutter modes
-  Support for reading the detectors with 16-bit or 32-bit depth
-  Saving files using the Andor SDK and/or with the standard
   areaDetector plugins
-  Change the ADC sampling speed (0.05MHz and 2.5MHz on the iKon) and
   the Vertical Shift Period
-  Set a region of interest (a smaller region can be read out faster)
-  Set and monitor the CCD temperature
-  Electron Multiplying (EM) Gain on supported detectors
-  Support for selecting between Full Vertical Binning (FVB) and Image
   readout modes
-  Support for Frame Transfer mode

The Andor module includes a separate driver to control the Andor
Shamrock spectrographs. If the detector data is saved in the Princeton
Instruments SPE file format using the Andor driver then it will include
the Shamrock wavelength calibration information. No other file formats
support saving the calibration.

This driver inherits from :doc:`../ADCore/ADDriver`
It implements many of the parameters in
`asynNDArrayDriver.h <../areaDetectorDoxygenHTML/asyn_n_d_array_driver_8h.html>`__
and in
`ADArrayDriver.h <../areaDetectorDoxygenHTML/_a_d_driver_8h.html>`__. It
also implements a number of parameters that are specific to the Andor
detectors. The `Andor class
documentation <../areaDetectorDoxygenHTML/class_andor_c_c_d.html>`__
describes this class in detail.

This document does not attempt to explain the meaning of the
Andor-specific parameters. The Andor Software Development Kit
documentation provides this detailed information. Andor does not allow
me to redistribute the SDK documentation as part of areaDetector. It
must be obtained from `Andor's Web
site <http://www.andor.com/scientific-software/software-development-kit/andor-sdk>`__.

areaDetector includes the header and library files required to build the
andor driver on any Linux or Windows computer. However, it does not
include the shareable libraries, DLLs or drivers to actually run a
detector. Those must be obtained from Andor, either by purchasing their
SDK or their Solis application software. On Windows the path to the
directory containing the Andor DLLs from the SDK or Solis must be added
to the PATH environment variable when running the areaDetector IOC. On
Linux the path to the directory containing the Andor shareable libraries
from the SDK must be added to the LD_LIBRARY_PATH environment variable
when running the areaDetector IOC.

NOTE: When using the Shamrock spectrograph on Windows the following DLLs
must actually be copied from the SDK directory to the current working
directory from which the IOC application is being run, e.g.
iocBoot/iocAndor.

-  atmcd32d.dll
-  ShamrockCIF.dll
-  atshamrock.dll

This is a rather strange requirement of the Andor Shamrock SDK, which
will hopefully be fixed by them in a future release.

.. note:: When using SDK version >= 2.102.30000.0 on Linux one must make
          sure that libUSBI2C-[ARCH].so.[VERSION] is installed as part of support
          into libUSBI2C.so and libUSBI2C.so.2.

Also `libd2xx_table.so` might be needed to get Shamrock communicating. See
`andorSupport/ftdi_table.c` for more.

Compile with:

::

       $ gcc -fpic -shared -Wl,-soname,libd2xx_table.so -o libd2xx_table.so ftdi_table.c
     

Place the `libd2xx_table.so` into the folder with the rest of SDK support
libraries.

Implementation of standard driver parameters
--------------------------------------------

The following table describes how the Andor driver implements some of
the standard driver parameters.

.. raw:: html

  <table class="table table-bordered"> 
    <tbody>
      <tr>
        <td align="center" colspan="3">
          <b>Implementation of Parameters in asynNDArrayDriver.h and ADDriver.h, and EPICS Record
            Definitions in ADBase.template and NDFile.template</b></td>
      </tr>
      <tr>
        <th>
          Parameter index variable</th>
        <th>
          EPICS record name</th>
        <th>
          Description</th>
      </tr>
      <tr>
        <td>
          ADTriggerMode</td>
        <td>
          $(P)$(R)TriggerMode<br />
          $(P)$(R)TriggerMode_RBV</td>
        <td>
          Sets the trigger mode for the detector. Options are:
          <ul>
            <li>Internal</li>
            <li>External</li>
            <li>External Start</li>
            <li>External Exposure</li>
            <li>External FVP</li>
            <li>Software</li>
          </ul>
        </td>
      </tr>
      <tr>
        <td>
          ADImageMode</td>
        <td>
          $(P)$(R)ImageMode<br />
          $(P)$(R)ImageMode_RBV</td>
        <td>
          Sets the image mode for the detector. Options are:
          <ul>
            <li>Single</li>
            <li>Multiple</li>
            <li>Continuous</li>
            <li>Fast Kinetics</li>
          </ul>
          The relation of ImageMode to the Andor acquisition modes are given in the table
          below. </td>
      </tr>
      <tr>
        <td>
          ADNumExposures</td>
        <td>
          $(P)$(R)NumExposures<br />
          $(P)$(R)NumExposures_RBV</td>
        <td>
          Sets the number of accumulations (performed in software in Andor's driver) in Single
          and Multiple modes</td>
      </tr>
      <tr>
        <td>
          ADNumImages</td>
        <td>
          $(P)$(R)NumImages<br />
          $(P)$(R)NumImages_RBV</td>
        <td>
          Sets the number of images to take in multiple (Kinetics Series) mode </td>
      </tr>
      <tr>
        <td>
          ADAcquirePeriod</td>
        <td>
          $(P)$(R)AcquirePeriod<br />
          $(P)$(R)AcquirePeriod_RBV</td>
        <td>
          Sets the time between images in Multiple (Kinetics Series) and Continuous (Run Till
          Abort) modes</td>
      </tr>
      <tr>
        <td>
          ADGain</td>
        <td>
          $(P)$(R)Gain<br />
          $(P)$(R)Gain_RBV</td>
        <td>
          Sets the pre-amp gain of the detector. For the Andor driver the Gain is treated
          as an integer index into the supported gain table of the specific detector. The
          list of supported gains for the detector gain be found by typing "asynReport 1,ANDOR"
          at the IOC prompt. For example, on the iKon-M the relationship is:
          <ul>
            <li>Gain=0 Andor gain=1.0</li>
            <li>Gain=1 Andor gain=2.0</li>
            <li>Gain=2 Andor gain=4.0</li>
          </ul>
        </td>
      </tr>
      <tr>
        <td>
          NDDataType</td>
        <td>
          $(P)$(R)DataType<br />
          $(P)$(R)DataType_RBV</td>
        <td>
          Sets data type for reading out the detector. Allowed values are:
          <ul>
            <li>UInt16</li>
            <li>UInt32</li>
          </ul>
          UInt16 can be used when reading out a 16-bit detector with NumExposures=1, (i.e.
          without accumulations), or when one can be sure that multiple accumulations will
          not overflow 16 bits. UInt32 should be used for 32-bit detectors or when multiple
          accumulations could cause 16-bit overflow. </td>
      </tr>
      <tr>
        <td>
          ADTemperature</td>
        <td>
          $(P)$(R)Temperature<br />
          $(P)$(R)Temperature_RBV</td>
        <td>
          Sets the setpoint temperature of the CCD (-120C to 20C) </td>
      </tr>
      <tr>
        <td>
          ADTemperatureActual</td>
        <td>
          $(P)$(R)TemperatureActual</td>
        <td>
          Reads the actual temperature of the CCD</td>
      </tr>
      <tr>
        <td>
          NDFileFormat</td>
        <td>
          $(P)$(R)FileFormat<br />
          $(P)$(R)FileFormat_RBV</td>
        <td>
          Selects the file format for saving files with the Andor driver. Choices are:
          <ul>
            <li>TIFF</li>
            <li>BMP</li>
            <li>SIF</li>
            <li>EDF</li>
            <li>RAW</li>
            <li>FITS</li>
            <li>SPE</li>
          </ul>
          All of the file formats except SPE are written by the Andor SDK. The SPE file format
          is written directly by the driver. It uses version 3.0 of the SPE format, which
          includes XML metadata after the image data. Only the SPE format is able to save
          the wavelength calibration from the Shamrock spectrographs. </td>
      </tr>
    </tbody>
  </table>

The following table shows the relationship of ImageMode to the Andor acquisition
modes, and the meaning of NumExposures and NumImages.

.. raw:: html

  <table class="table table-bordered"> 
    <tbody>
      <tr>
        <td align="center" colspan="7">
          <b>Relationship of ImageMode to the Andor acquisition modes, and the meaning of NumExposures
            and NumImages.</b></td>
      </tr>
      <tr>
        <th>
          ImageMode</th>
        <th>
          NumExposures</th>
        <th>
          AcquireTime</th>
        <th>
          AndorAccumulatePeriod</th>
        <th>
          NumImages</th>
        <th>
          AcquirePeriod</th>
        <th>
          Andor acquisition mode</th>
      </tr>
      <tr>
        <td>
          Single</td>
        <td>
          1</td>
        <td>
          Sets exposure time</td>
        <td>
          Not applicable</td>
        <td>
          Not applicable</td>
        <td>
          Not applicable</td>
        <td>
          Single Scan</td>
      </tr>
      <tr>
        <td>
          Single</td>
        <td>
          &gt;1 Sets number of accumulations per image.</td>
        <td>
          Sets exposure time per accumulation</td>
        <td>
          Sets accumulation period (cycle time)</td>
        <td>
          Not applicable</td>
        <td>
          Not applicable</td>
        <td>
          Accumulate</td>
      </tr>
      <tr>
        <td>
          Multiple</td>
        <td>
          Sets number of accumulations per image</td>
        <td>
          Sets exposure time per accumulation</td>
        <td>
          Sets accumulation period if NumExposures &gt; 1</td>
        <td>
          Sets number of images </td>
        <td>
          Sets time between images (cycle time)</td>
        <td>
          Kinetic Series</td>
      </tr>
      <tr>
        <td>
          Continuous</td>
        <td>
          Not applicable</td>
        <td>
          Sets exposure time per image</td>
        <td>
          Not applicable</td>
        <td>
          Not applicable</td>
        <td>
          Sets time between images (cycle time)</td>
        <td>
          Run Till Abort</td>
      </tr>
      <tr>
        <td>
          Fast Kinetics</td>
        <td>
          Not applicable</td>
        <td>
          Sets exposure time per sub-area</td>
        <td>
          Not applicable</td>
        <td>
          Controls number of sub-area exposures, each being followed by a vertical shift of
          SizeY. MinY controls the offset of the first row from the bottom of the CCD. SizeY
          controls the sub-area height. BinX and BinY control the horizontal and vertical
          binning.</td>
        <td>
          Not applicable</td>
        <td>
          Fast Kinetics</td>
      </tr>
    </tbody>
  </table>

Andor specific parameters
-------------------------

The Andor driver implements the following parameters in addition to
those in asynNDArrayDriver.h and ADDriver.h.

.. raw:: html

  <table class="table table-bordered"> 
    <tbody>
      <tr>
        <td align="center" colspan="7">
          <b>Parameter Definitions in andorCCD.h and EPICS Record Definitions in andorCCD.template</b>
        </td>
      </tr>
      <tr>
        <th>
          Parameter index variable</th>
        <th>
          asyn interface</th>
        <th>
          Access</th>
        <th>
          Description</th>
        <th>
          drvInfo string</th>
        <th>
          EPICS record name</th>
        <th>
          EPICS record type</th>
      </tr>
      <tr>
        <td>
          AndorCoolerParam</td>
        <td>
          asynInt32</td>
        <td>
          R/W</td>
        <td>
          Turn the CCD cooler on and off</td>
        <td>
          ANDOR_COOLER</td>
        <td>
          AndorCooler<br />
          AndorCooler_RBV</td>
        <td>
          bo<br />
          bi</td>
      </tr>
      <tr>
        <td>
          AndorTempStatusMessage</td>
        <td>
          asynOctet</td>
        <td>
          R/O</td>
        <td>
          Temperature status message.</td>
        <td>
          ANDOR_TEMP_STAT</td>
        <td>
          AndorTempStatus_RBV</td>
        <td>
          waveform</td>
      </tr>
      <tr>
        <td>
          AndorMessage</td>
        <td>
          asynOctet</td>
        <td>
          R/O</td>
        <td>
          Other status message.</td>
        <td>
          ANDOR_MESSAGE</td>
        <td>
          AndorMessage_RBV</td>
        <td>
          waveform</td>
      </tr>
      <tr>
        <td>
          AndorShutterMode</td>
        <td>
          asynInt32</td>
        <td>
          R/W</td>
        <td>
          Selects the Andor shutter mode. Choices are:
          <ul>
            <li>Full Auto</li>
            <li>Always Open</li>
            <li>Always Closed</li>
            <li>Open for FVB</li>
            <li>Open for Any</li>
          </ul>
        </td>
        <td>
          ANDOR_SHUTTER_MODE</td>
        <td>
          AndorShutterMode</td>
        <td>
          mbbo</td>
      </tr>
      <tr>
        <td>
          AndorShutterExTTL</td>
        <td>
          asynInt32</td>
        <td>
          R/W</td>
        <td>
          Selects the TTL polarity of an external shutter. Choices are:
          <ul>
            <li>Low To Open</li>
            <li>High To Open</li>
          </ul>
        </td>
        <td>
          ANDOR_SHUTTER_EXTTL</td>
        <td>
          AndorShutterExTTL</td>
        <td>
          bo</td>
      </tr>
      <tr>
        <td>
          AndorPALFileName</td>
        <td>
          asynOctet</td>
        <td>
          R/W</td>
        <td>
          Path and Filename of pallette file (used for TIFF and BMP file colours) (255 chars
          max). </td>
        <td>
          ANDOR_PAL_FILE_PATH</td>
        <td>
          PALFilePath</td>
        <td>
          waveform</td>
      </tr>
      <tr>
        <td>
          AndorAdcSpeed</td>
        <td>
          asynInt32</td>
        <td>
          R/W</td>
        <td>
          Switch between the slow (low noise) ADC and the fast ADC. Choices are:
          <ul>
            <li>0.05 MHz</li>
            <li>2.5 MHz</li>
          </ul>
        </td>
        <td>
          ANDOR_ADC_SPEED</td>
        <td>
          AndorADCSpeed<br />
          AndorADCSpeed_RBV</td>
        <td>
          mbbo<br />
          mbbi</td>
      </tr>
      <tr>
        <td>
          AndorAccumulatePeriod</td>
        <td>
          asynFloat64</td>
        <td>
          R/W</td>
        <td>
          Controls the period between accumulations when ImageMode=Single or Multiple and
          NumExposures&gt;1. NOTE: Some Andor detectors (including the iKon) only support
          a single period when doing multiple accumulations in kinetic series mode. For these
          cameras ANDOR_ACCUMULATE_PERIOD has no effect, ACQUIRE_PERIOD determines the time
          between accumulations, and the time between images is 0, i.e. the next image starts
          as soon as the previous one is complete.</td>
        <td>
          ANDOR_ACCUMULATE_PERIOD</td>
        <td>
          AndorAccumulatePeriod<br />
          AndorAccumulatePeriod_RBV</td>
        <td>
          ao<br />
          ai</td>
      </tr>
      <tr>
        <td>
          AndorAccumulatePeriodActual</td>
        <td>
          asynFloat64</td>
        <td>
          R/O</td>
        <td>
          Reads the actual value of AndorAccumulatePeriod, which may differ from the requested
          value due to timing limitations of the detector. </td>
        <td>
          ANDOR_ACCUMULATE_PERIOD_ACTUAL</td>
        <td>
          AndorAccumulatePeriodActual</td>
        <td>
          ai</td>
      </tr>
      <tr>
        <td>
          AndorAcquireTimeActual</td>
        <td>
          asynFloat64</td>
        <td>
          R/O</td>
        <td>
          Reads the actual value of ADAcquireTime, which may differ from the requested value
          due to timing limitations of the detector. </td>
        <td>
          ANDOR_ACQUIRE_TIME_ACTUAL</td>
        <td>
          AndorAcquireTimeActual</td>
        <td>
          ai</td>
      </tr>
      <tr>
        <td>
          AndorAcquirePeriodActual</td>
        <td>
          asynFloat64</td>
        <td>
          R/O</td>
        <td>
          Reads the actual value of ADAcquirePeriod, which may differ from the requested value
          due to timing limitations of the detector. </td>
        <td>
          ANDOR_ACQUIRE_PERIOD_ACTUAL</td>
        <td>
          AndorAcquirePeriodActual</td>
        <td>
          ai</td>
      </tr>
      <tr>
        <td>
          AndorBaselineClamp</td>
        <td>
          asynInt32</td>
        <td>
          R/W</td>
        <td>
          Enable or disable the baseline clamp option.</td>
        <td>
          ANDOR_BASELINE_CLAMP</td>
        <td>
          AndorBaselineClamp<br />
          AndorBaselineClamp_RBV</td>
        <td>
          bo<br />
          bi</td>
      </tr>
      <tr>
        <td>
          AndorEMGain</td>
        <td>
          asynInt32</td>
        <td>
          R/W</td>
        <td>
          Controls the Electron Multiplying (EM) Gain level on supported detectors. The valid
          range depends on the value of AndorEMGainMode and the detector temperature. For
          cameras that do not support EM Gain, AndorEMGain has no effect.</td>
        <td>
          ANDOR_EM_GAIN</td>
        <td>
          AndorEMGain<br />
          AndorEMGain_RBV</td>
        <td>
          ao<br />
          ai</td>
      </tr>
      <tr>
        <td>
          AndorEMGainMode</td>
        <td>
          asynInt32</td>
        <td>
          R/W</td>
        <td>
          Sets the EM Gain mode on supported detectors. Choices are:
          <ul>
            <li>8 bit DAC</li>
            <li>12 bit DAC</li>
            <li>Linear Mode</li>
            <li>Real EM Gain</li>
          </ul>
          For cameras that do not support EM Gain, AndorEMGainMode has no effect.</td>
        <td>
          ANDOR_EM_GAIN_MODE</td>
        <td>
          AndorEMGainMode<br />
          AndorEMGainMode_RBV</td>
        <td>
          mbbo<br />
          mbbi</td>
      </tr>
      <tr>
        <td>
          AndorEMGainAdvanced</td>
        <td>
          asynInt32</td>
        <td>
          R/W</td>
        <td>
          Enables access to higher EM Gain levels. Choices are:
          <ul>
            <li>Disabled</li>
            <li>Enabled</li>
          </ul>
          For cameras that do not support EM Gain, AndorEMGainAdvanced has no effect. NOTE:
          Before using higher levels, you should ensure that light levels do not exceed the
          regime of tens of photons per pixel, otherwise accelerated ageing of the sensor
          can occur. </td>
        <td>
          ANDOR_EM_GAIN_ADVANCED</td>
        <td>
          AndorEMGainAdvanced<br />
          AndorEMGainAdvanced_RBV</td>
        <td>
          bo<br />
          bi</td>
      </tr>
      <tr>
        <td>
          AndorReadOutMode</td>
        <td>
          asynInt32</td>
        <td>
          R/W</td>
        <td>
          Switch between the readout modes. Choices are:
          <ul>
            <li>Full Vertical Binning (FVB)</li>
            <li>Image</li>
          </ul>
        </td>
        <td>
          ANDOR_READOUT_MODE</td>
        <td>
          AndorReadOutMode<br />
          AndorReadOutMode_RBV</td>
        <td>
          mbbo<br />
          mbbi</td>
      </tr>
      <tr>
        <td>
          AndorFTMode</td>
        <td>
          asynInt32</td>
        <td>
          R/W</td>
        <td>
          Set Frame Transfer mode. Choices are:
          <ul>
            <li>Disabled</li>
            <li>Enabled</li>
          </ul>
          Note: Only available on supported CCDs.
        </td>
        <td>
          ANDOR_FT_MODE</td>
        <td>
          AndorFTMode<br />
          AndorFTMode_RBV</td>
        <td>
          bo<br />
          bi</td>
      </tr>
      <tr>
        <td>
          AndorVSPeriod</td>
        <td>
          asynInt32</td>
        <td>
          R/W</td>
        <td>
          Sets Vetical Shift Period, in units of microseconds per pixel shift.<br/>
          Choices are constructed at runtime. For example, the choices for an iDus are:
          <ul>
            <li>4.25 us</li>
            <li>8.25 us</li>
            <li>16.25 us</li>
            <li>32.25 us</li>
            <li>64.25 us</li>
          </ul>
        <td>
          ANDOR_VS_PERIOD</td>
        <td>
          AndorVSPeriod<br />
          AndorVSPeriod_RBV</td>
        <td>
          mbbo<br />
          mbbi</td>
      </tr>
    </tbody>
  </table>


Unsupported standard driver parameters
--------------------------------------

ColorMode, ReverseX, and ReverseY are currently not supported.

Shamrock spectrograph control
-----------------------------

The Andor module also includes a driver for the Andor Shamrock
spectrographs. This driver allows control of the grating, central
wavelength, and slit sizes.

.. raw:: html

  <table class="table table-bordered"> 
    <tbody>
      <tr>
        <td align="center" colspan="7">
          <b>Parameter Definitions in shamrock.cpp and EPICS Record Definitions in shamrock.template</b>
        </td>
      </tr>
      <tr>
        <th>
          Parameter index variable</th>
        <th>
          asyn interface</th>
        <th>
          Access</th>
        <th>
          Description</th>
        <th>
          drvInfo string</th>
        <th>
          EPICS record name</th>
        <th>
          EPICS record type</th>
      </tr>
      <tr>
        <td>
          SRGrating</td>
        <td>
          asynInt32</td>
        <td>
          R/W</td>
        <td>
          Selects the grating to use</td>
        <td>
          SR_GRATING</td>
        <td>
          Grating<br />
          Grating_RBV</td>
        <td>
          mbbo<br />
          mbbi</td>
      </tr>
      <tr>
        <td>
          SRGratingExists</td>
        <td>
          asynInt32</td>
        <td>
          R/O</td>
        <td>
          Flag indicating if a grating is present</td>
        <td>
          SR_GRATING_EXISTS</td>
        <td>
          GratingExists[N], N=1-3</td>
        <td>
          bi</td>
      </tr>
      <tr>
        <td>
          SRWavelength</td>
        <td>
          asynFloat64</td>
        <td>
          R/W</td>
        <td>
          Selects the central wavelength</td>
        <td>
          SR_WAVELENGTH</td>
        <td>
          Wavelength<br />
          Wavelength_RBV</td>
        <td>
          ao<br />
          ai</td>
      </tr>
      <tr>
        <td>
          SRMinWavelength</td>
        <td>
          asynFloat64</td>
        <td>
          R/O</td>
        <td>
          The minimum wavelength of the current configuration (ADDR=0) or the minimum wavelength
          of grating N (N=1-3)</td>
        <td>
          SR_MIN_WAVELENGTH</td>
        <td>
          MinWavelength, MinWavelength[N], N=1-3</td>
        <td>
          ai</td>
      </tr>
      <tr>
        <td>
          SRMaxWavelength</td>
        <td>
          asynFloat64</td>
        <td>
          R/O</td>
        <td>
          The maximum wavelength of the current configuration or the maximum wavelength of
          grating N (N=1-3)</td>
        <td>
          SR_MAX_WAVELENGTH</td>
        <td>
          MaxWavelength, MaxWavelength[N], N=1-3</td>
        <td>
          ai</td>
      </tr>
      <tr>
        <td>
          SRSlitSize</td>
        <td>
          asynFloat64</td>
        <td>
          R/W</td>
        <td>
          The size of slit N, N=1-4. The slits are numbered as follows:<br />
          <ol>
            <li>Input slit side</li>
            <li>Input slit direct</li>
            <li>Output slit side</li>
            <li>Output slit direct</li>
          </ol>
        </td>
        <td>
          SR_SLIT_SIZE</td>
        <td>
          SlitSize[N], N=1-4<br />
          SlitSize[N]_RBV</td>
        <td>
          ao<br />
          ai</td>
      </tr>
      <tr>
        <td>
          SRSlitExists</td>
        <td>
          asynInt32</td>
        <td>
          R/O</td>
        <td>
          Flag indicating if a slit is present</td>
        <td>
          SR_SLIT_EXISTS</td>
        <td>
          SlitExists[N], N=1-4</td>
        <td>
          bi</td>
      </tr>
      <tr>
        <td>
          SRCalibration</td>
        <td>
          asynFloat32Array</td>
        <td>
          R/O</td>
        <td>
          Array containing the wavelength calibration of each X pixel of the detector in nm.
        </td>
        <td>
          SR_CALIBRATION</td>
        <td>
          Calibration</td>
        <td>
          bi</td>
      </tr>
    </tbody>
  </table>

Usage
-----

Always use channel access put callback when setting parameters.

If any of the parameters set are out of range or fail in some way, then
the PV will be put into alarm state. This should be checked after every
PV set.

An example palette file for a TIFF file is GREY.PAL in the iocAndor
directory.

Configuration
-------------

The Andor driver is created with the andorCCDConfig command, either from
C/C++ or from the EPICS IOC shell.

::

   int andorCCDConfig(const char *portName,
                   int maxBuffers, size_t maxMemory,
                   const char* installPath,
                   int priority, int stackSize)
     

The Shamrock driver is created with the shamrockConfig command, either
from C/C++ or from the EPICS IOC shell.

::

   int shamrockConfig(const char *portName, 
                             int shamrockId, const char *iniPath, 
                             int priority, int stackSize)
     

For details on the meaning of the parameters to this function refer to
the detailed documentation on the andorCCDConfig function in the
`shamrock.cpp
documentation <../areaDetectorDoxygenHTML/shamrock8cpp.html>`__ and in the
documentation for the constructor for the `shamrock
class <../areaDetectorDoxygenHTML/class_shamrock.html>`__.

There an example IOC boot directory and startup script
:doc:`st_cmd` provided with
areaDetector.

MEDM screens
------------

The following shows the MEDM screen that is used to control the Andor
detector. Note that the general purpose screen ADBase.adl can be used,
but it exposes a few controls that are not applicable to the Andor, and
lacks some fields that are important for the Andor.

``Andor.adl`` is the main screen used to control the Andor driver.

.. figure:: Andor.png
    :align: center

The following shows the MEDM screen that is used to save files directly
with the Andor detector.

.. figure:: AndorFile.png
    :align: center

The following shows the MEDM screen that is used to control the Shamrock
spectrograph.

.. figure:: Shamrock.png
    :align: center

Restrictions
------------

The following are known restrictions of the Andor driver. These should
be fixed in a future release.

-  No support for detector output signals (trigger and gate).
-  Some Andor detectors (including the iKon) only support a single
   period when doing multiple accumulations in kinetic series mode. For
   these cameras ANDOR_ACCUMULATE_PERIOD has no effect, ACQUIRE_PERIOD
   determines the time between accumulations, and the time between
   images is 0, i.e. the next image starts as soon as the previous one
   is complete.
-  Saving files using the Andor driver in Multiple and Continuous modes
   results in errors because the Andor SDK functions won't save files if
   acquisition is in progress. Saving files in Single mode and Fast
   Kinetics mode works fine.
-  Trigger modes have not been tested.
-  The Shamrock wavelength calibration is only saved in SPE files. The
   Andor SDK file writers do not save the calibration, and it is not
   possible to pass the calibration to other file plugins as an
   attribute because array attributes are not currently supported in
   areaDetector.
-  Single-Track, Multi-Track and Random-Track readout modes are not yet
   supported.

