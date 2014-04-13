TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

LIBRARY_IOC_WIN32 += andorCCD
LIBRARY_IOC_Linux += andorCCD
LIB_SRCS += andorCCD.cpp
LIB_SRCS_WIN32 += shamrock.cpp
ifeq (win32-x86, $(findstring win32-x86, $(T_A)))
LIB_LIBS += ATMCD32M
else ifeq (windows-x64, $(findstring windows-x64, $(T_A)))
LIB_LIBS += atmcd64m
endif

DATA+=GREY.PAL

DBD += andorCCDSupport.dbd
DBD += shamrockSupport.dbd

include $(ADCORE)/ADApp/commonLibraryMakefile

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

