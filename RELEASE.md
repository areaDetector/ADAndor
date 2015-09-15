ADAndor Releases
==================

The latest untagged master branch can be obtained at
https://github.com/areaDetector/ADAndor.

Tagged source code and pre-built binary releases prior to R2-0 are included
in the areaDetector releases available via links at
http://cars.uchicago.edu/software/epics/areaDetector.html.

Tagged source code releases from R2-0 onward can be obtained at 
https://github.com/areaDetector/ADAndor/releases.

Tagged prebuilt binaries from R2-0 onward can be obtained at
http://cars.uchicago.edu/software/pub/ADAndor.

The versions of EPICS base, asyn, and other synApps modules used for each release can be obtained from 
the EXAMPLE_RELEASE_PATHS.local, EXAMPLE_RELEASE_LIBS.local, and EXAMPLE_RELEASE_PRODS.local
files respectively, in the configure/ directory of the appropriate release of the 
[top-level areaDetector](https://github.com/areaDetector/areaDetector) repository.


Release Notes
=============

R2-4 (September 15, 2015)
* Updated autoconverted .edl, .ui, and .opi files.

R2-3 (16-April-2015)
----
* Upgraded the Andor SDK version used to 2.99.3 on both Windows and Linux.
* Changed st.cmd to be compatible with ADCore R2-2. 
* Added support for mirror flippers on Shamrock spectrograph (thanks to Matthew Moore and Russell Woods).

R2-2 (12-March-2015)
----
* Upgraded the Andor SDK version used to 2.98.3 on both Windows and Linux. 
  - R2-0 through R2-1-1 used 2.90 on Windows and 2.84 on Linux.
* Added support for the Shamrock spectrographs on Linux. 
  However, andorDoc.html and the example st.cmd file were not updated
  to reflect this change, so they indicated that the Shamrock was not 
  supported on Linux.

R2-1-1 (13-Jul-2014)
----
* Added support for the Shamrock spectrographs on 64-bit Windows. 
  The notes for R2-1 are incorrect, there is 64-bit support for the Shamrock, 
  it is just a little hard to find.

R2-1 (15-Apr-2014)
----
* Added support for the Shamrock spectrographs.  
  This supports control of grating selection, center wavelength, and slit sizes.
  It is a separate driver, and is only supported on 32-bit Windows because Andor does not provide a 
  Shamrock SDK for 64-bit Windows or Linux.
* Added support for saving data in Princeton Instruments V3.0 SPE file format with XML footer.
  This is used to save calibration information from the Shamrock spectrometer.
  None of the other formats will save calibration information from the SDK, although they do from Solis.

R2-0 (24-Mar-2014)
----
* Moved the repository to [Github](https://github.com/areaDetector/ADAndor).
* Re-organized the directory structure to separate the driver library from the example IOC application.

R1-9-1 and earlier
------------------
Release notes are part of the
[areaDetector Release Notes](http://cars.uchicago.edu/software/epics/areaDetectorReleaseNotes.html).

Future Releases
===============
* Upgrade to Release 2.98 of SDK
* dataTask should check while (!mExiting)
