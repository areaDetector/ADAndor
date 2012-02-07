TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

LIBRARY_IOC_WIN32 += andorCCD
LIBRARY_IOC_Linux += andorCCD
LIB_SRCS += andorCCD.cpp

DATA+=GREY.PAL

# <name>.dbd will be created from <name>Include.dbd
DBD += andorCCDApp.dbd
DBD += andorCCDSupport.dbd

# Note: we do things below in an arch-dependent way because we must built the application
# without GraphicsMagick for windows-x64.  This is because of a conflict with the Andor
# library (SetImage function) that only occurs on windows-x64.  So we build all
# other archs with commonDriverSupport.dbd, but windows-x64 does not use that.
# We have a separate dbd file for windows-x64, and the application has a different name.
# We also do default STATIC_BUILD=YES on Windows but dynamic on Linux because Andor only 
# provides a .so file on Linux.

# Add locally compiled object code
PROD_LIBS += andorCCD

ifeq (win32-x86, $(findstring win32-x86, $(T_A)))
PROD_SRCS += andorCCDApp_registerRecordDeviceDriver.cpp andorCCDAppMain.cpp
PROD_IOC  += andorCCDApp
PROD_LIBS += ATMCD32M
#PROD_IOC += accumTest
#accumTest_SRCS += accumTest.c
#accumTest_LIBS += ATMCD32M 

else ifeq (windows-x64, $(findstring windows-x64, $(T_A)))
PROD_SRCS += andorCCDAppWin64_registerRecordDeviceDriver.cpp andorCCDAppMain.cpp
DBD       += andorCCDAppWin64.dbd
PROD_IOC  += andorCCDAppWin64
PROD_LIBS += atmcd64m

else ifeq (linux-x86_64, $(findstring linux-x86_64, $(T_A)))
PROD_SRCS += andorCCDApp_registerRecordDeviceDriver.cpp andorCCDAppMain.cpp
PROD_IOC  += andorCCDApp
PROD_LIBS += andor
STATIC_BUILD=NO

else ifeq (linux-x86, $(findstring linux-x86, $(T_A)))
PROD_SRCS += andorCCDApp_registerRecordDeviceDriver.cpp andorCCDAppMain.cpp
PROD_IOC  += andorCCDApp
PROD_LIBS += andor
STATIC_BUILD=NO

endif

include $(TOP)/ADApp/commonDriverMakefile

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

