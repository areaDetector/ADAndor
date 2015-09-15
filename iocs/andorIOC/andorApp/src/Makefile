TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

# Note: we must built the application without GraphicsMagick.
# This is because of a conflict with the Andor library (SetImage function) 
# It leads to a build error on windows-x64, but to a run-time error on Linux.
USE_GRAPHICSMAGICK=NO

PROD_NAME = andorCCDApp
PROD_IOC_WIN32  += $(PROD_NAME)
PROD_IOC_Linux  += $(PROD_NAME)

# <name>.dbd will be created from <name>Include.dbd
DBD += $(PROD_NAME).dbd
$(PROD_NAME)_DBD += andorCCDSupport.dbd
$(PROD_NAME)_DBD += shamrockSupport.dbd

$(PROD_NAME)_SRCS += $(PROD_NAME)_registerRecordDeviceDriver.cpp $(PROD_NAME)Main.cpp

# Add locally compiled object code
PROD_LIBS       += andorCCD

ifeq (win32-x86, $(findstring win32-x86, $(T_A)))
  PROD_LIBS  += atmcd32m

else ifeq (windows-x64, $(findstring windows-x64, $(T_A)))
  # Note: we must built the application without GraphicsMagick for windows-x64.  
  # This is because of a conflict with the Andor library (SetImage function) 
  # that only occurs on windows-x64.
  USE_GRAPHICSMAGICK=NO
  PROD_LIBS  += atmcd64m
endif
PROD_LIBS_WIN32 += ShamrockCIFm

# Test application
#PROD_IOC_WIN32 += test_xml
#PROD_IOC_Linux += test_xml
#test_xml += test_xml.cpp

# Note: we use PROD_SYS_LIBS because we are building statically but this library is only provided in dynamic
# format (.so)
PROD_SYS_LIBS_Linux += andor
PROD_SYS_LIBS_Linux += shamrockcif

include $(ADCORE)/ADApp/commonDriverMakefile

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

