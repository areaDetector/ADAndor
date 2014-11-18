TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

LIBRARY_IOC_WIN32 += andorCCD
LIBRARY_IOC_Linux += andorCCD
LIB_SRCS += andorCCD.cpp
LIB_SRCS += shamrock.cpp
LIB_LIBS_WIN32 += atmcd64m
LIB_LIBS_WIN32 += ShamrockCIFm

DATA+=GREY.PAL

DBD += andorCCDSupport.dbd
DBD += shamrockSupport.dbd

include $(ADCORE)/ADApp/commonLibraryMakefile

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

