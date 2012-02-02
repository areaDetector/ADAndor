TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

LIBRARY_IOC_WIN32 += andorCCD
LIBRARY_IOC_Linux += andorCCD
LIB_SRCS += andorCCD.cpp

DATA+=GREY.PAL

PROD_IOC_WIN32 += andorCCDApp
PROD_IOC_Linux += andorCCDApp

# <name>.dbd will be created from <name>Include.dbd
DBD += andorCCDApp.dbd
DBD += andorCCDSupport.dbd

# <name>_registerRecordDeviceDriver.cpp will be created from <name>.dbd
PROD_SRCS += andorCCDApp_registerRecordDeviceDriver.cpp andorCCDAppMain.cpp

# Add locally compiled object code
PROD_LIBS += andorCCD

ifeq (win32-x86, $(findstring win32-x86, $(T_A)))
PROD_LIBS += ATMCD32M

else ifeq (windows-x64, $(findstring windows-x64, $(T_A)))
PROD_LIBS += atmcd64m

else ifeq (linux-x86_64, $(findstring linux-x86_64, $(T_A)))
PROD_LIBS += andor
# Cannot do STATIC_BUILD on Linux because Andor only provides .so file
STATIC_BUILD=NO

else ifeq (linux-x86, $(findstring linux-x86, $(T_A)))
PROD_LIBS += andor
STATIC_BUILD=NO

endif

include $(TOP)/ADApp/commonDriverMakefile

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

