TOP=../..

include $(TOP)/configure/CONFIG

# -------------------------------
# Build an Diamond Support Module
# -------------------------------

LIBRARY_IOC += andorCCD

DATA+=GREY.PAL

USR_INCLUDES += -I$(ANDORCCDSDK)/sdk_include

# xxxRecord.dbd will be installed into <top>/dbd
DBD += andorCCD.dbd
DBD += andorCCDSupport.dbd

andorCCD_DBD += base.dbd
andorCCD_DBD += ADSupport.dbd
andorCCD_DBD += asyn.dbd
andorCCD_DBD += calcSupport.dbd

# The following are compiled and added to the support library
andorCCD_SRCS += drvAndorCCDEpics.c
andorCCD_SRCS += AndorCCD.cpp

andorCCD_LIBS += asyn
andorCCD_LIBS += NDPlugin 
andorCCD_LIBS += ADBase
andorCCD_LIBS += calc

andor_DIR += $(ANDORCCDSDK)/sdk_libs/

andorCCD_LIBS += andor

# We need to link against the EPICS Base libraries
andorCCD_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES
