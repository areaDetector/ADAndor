// Dummy file for Linux

#include <stdio.h>
#include <iocsh.h>
#include <epicsExport.h>

static const iocshArg configArg0 = {"Port name", iocshArgString};
static const iocshArg configArg1 = {"shamrockId", iocshArgInt};
static const iocshArg configArg2 = {"iniPath", iocshArgString};
static const iocshArg configArg3 = {"priority", iocshArgInt};
static const iocshArg configArg4 = {"stackSize", iocshArgInt};
static const iocshArg * const configArgs[] = {&configArg0,
                                              &configArg1,
                                              &configArg2,
                                              &configArg3,
                                              &configArg4};
static const iocshFuncDef configShamrock = {"shamrockConfig", 5, configArgs};
static void configCallFunc(const iocshArgBuf *args)
{
    printf("ERROR: shamrockConfig is not supported on Linux\n");
}


static void shamrockRegister(void)
{
    iocshRegister(&configShamrock, configCallFunc);
}

extern "C" {
epicsExportRegistrar(shamrockRegister);
}

