TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

LIBRARY_IOC_WIN32 += andorCCD
LIBRARY_IOC_Linux += andorCCD
LIB_SRCS += andorCCD.cpp
LIB_SRCS += shamrock.cpp
ifeq (win32-x86, $(findstring win32-x86, $(T_A)))
LIB_LIBS_WIN32 += atmcd32m
else ifeq (windows-x64, $(findstring windows-x64, $(T_A)))
LIB_LIBS_WIN32 += atmcd64m
endif
LIB_LIBS_WIN32 += ShamrockCIFm

DATA+=GREY.PAL

DBD += andorCCDSupport.dbd
DBD += shamrockSupport.dbd

ifdef XML2_INCLUDE
  USR_INCLUDES += $(addprefix -I, $(XML2_INCLUDE))
endif

include $(ADCORE)/ADApp/commonLibraryMakefile

PROD_IOC_Linux += linuxTest
PROD_IOC_Linux += FKTest
PROD_IOC_Linux += accumTest
PROD_SYS_LIBS += andor

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

