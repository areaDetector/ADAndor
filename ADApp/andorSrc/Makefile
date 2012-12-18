TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

LIBRARY_IOC_WIN32 += andorCCD
LIBRARY_IOC_Linux += andorCCD
LIB_SRCS += andorCCD.cpp

DATA+=GREY.PAL

# We also do default STATIC_BUILD=YES on Windows but dynamic on Linux because Andor only 
# provides a .so file on Linux.

# Add locally compiled object code
PROD_LIBS += andorCCD

PROD_NAME = andorCCDApp
PROD_IOC_WIN32  += $(PROD_NAME)
PROD_IOC_Linux  += $(PROD_NAME)

# <name>.dbd will be created from <name>Include.dbd
DBD += $(PROD_NAME).dbd
$(PROD_NAME)_DBD += andorCCDSupport.dbd
DBD += andorCCDSupport.dbd

PROD_SRCS += andorCCDApp_registerRecordDeviceDriver.cpp andorCCDAppMain.cpp

ifeq (win32-x86, $(findstring win32-x86, $(T_A)))
PROD_LIBS += ATMCD32M

else ifeq (windows-x64, $(findstring windows-x64, $(T_A)))
# Note: we must built the application without GraphicsMagick for windows-x64.  
# This is because of a conflict with the Andor library (SetImage function) 
# that only occurs on windows-x64.
USE_GRAPHICSMAGICK=NO
PROD_LIBS += atmcd64m
endif

# Note: we use PROD_SYS_LIBS because we are building statically but this library is only provided in dynamic
# format (.so)
PROD_SYS_LIBS_Linux += andor

include $(TOP)/ADApp/commonDriverMakefile

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

