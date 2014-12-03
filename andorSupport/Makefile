TOP=..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

# Note, the files in this directory were manually copied from the Andor SDK directories 
# for Linux and Windows.  The soft links for Linux were manually created.

# The extension on ATMCD32D.h was changed from .H to .h because the EPICS build system does
# not recognize .H
INC += ATMCD32D.h
INC += atmcdLXd.h
INC += ShamrockCIF.h

ifeq (win32-x86, $(findstring win32-x86, $(T_A)))
LIB_INSTALLS_WIN32    += ../os/win32-x86/atmcd32m.lib
LIB_INSTALLS_WIN32    += ../os/win32-x86/ShamrockCIFm.lib

else ifeq (windows-x64, $(findstring windows-x64, $(T_A)))
LIB_INSTALLS_WIN32    += ../os/windows-x64/atmcd64m.lib
LIB_INSTALLS_WIN32    += ../os/windows-x64/ShamrockCIFm.lib

else ifeq (linux-x86_64, $(findstring linux-x86_64, $(T_A)))
LIB_INSTALLS_Linux += ../os/linux-x86_64/libandor.so.2
LIB_INSTALLS_Linux += ../os/linux-x86_64/libandor.so
LIB_INSTALLS_Linux += ../os/linux-x86_64/libshamrockcif.so.2
LIB_INSTALLS_Linux += ../os/linux-x86_64/libshamrockcif.so

else ifeq (linux-x86, $(findstring linux-x86, $(T_A)))
LIB_INSTALLS_Linux += ../os/linux-x86/libandor.so.2
LIB_INSTALLS_Linux += ../os/linux-x86/libandor.so
LIB_INSTALLS_Linux += ../os/linux-x86/libshamrockcif.so.2
LIB_INSTALLS_Linux += ../os/linux-x86/libshamrockcif.so

endif

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

