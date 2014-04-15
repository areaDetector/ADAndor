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

R2-0
----
* Moved the repository to [Github](https://github.com/areaDetector/ADAndor).
* Re-organized the directory structure to separate the driver library from the example IOC application.

R2-1
----
* Added support for the Shamrock spectrographs.  
  This supports control of grating selection, center wavelength, and slit sizes.
  It is a separate driver, and is only supported on Windows because Andor does not provide a 
  Shamrock SDK on Linux.
* Added support for saving data in Princeton Instruments V3.0 SPE file format with XML footer.
  This is used to save calibration information from the Shamrock spectrometer.
  None of the other formats will save calibration information from the SDK, although they do from Solis.

R1-9-1 and earlier
------------------
Release notes are part of the
[areaDetector Release Notes](http://cars.uchicago.edu/software/epics/areaDetectorReleaseNotes.html).


Future Releases
===============
* Add exiting_ flag; statusTask and dataTask should both do while (!exiting_)
