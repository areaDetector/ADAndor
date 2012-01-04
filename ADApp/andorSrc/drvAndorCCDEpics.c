
/**
 *
 * This is the EPICS dependent code for the Andor driver.
 * By making this separate file for the EPICS dependent code the driver itself
 * only needs libCom from EPICS for OS-independence.
 *
 */

#include <iocsh.h>
#include <drvSup.h>
#include <epicsExport.h>

#include "drvAndorCCD.h"


/* Code for iocsh registration */

/* andorCCDConfig */
static const iocshArg andorCCDConfigArg0 = {"Port name", iocshArgString};
static const iocshArg andorCCDConfigArg1 = {"maxBuffers", iocshArgInt};
static const iocshArg andorCCDConfigArg2 = {"maxMemory", iocshArgInt};
static const iocshArg andorCCDConfigArg3 = {"maxSizeX", iocshArgInt};
static const iocshArg andorCCDConfigArg4 = {"maxSizeY", iocshArgInt};
static const iocshArg * const andorCCDConfigArgs[] =  {&andorCCDConfigArg0,
						       &andorCCDConfigArg1,
						       &andorCCDConfigArg2,
                                                       &andorCCDConfigArg3,
                                                       &andorCCDConfigArg4};

static const iocshFuncDef configAndorCCD = {"andorCCDConfig", 5, andorCCDConfigArgs};
static void configAndorCCDCallFunc(const iocshArgBuf *args)
{
    andorCCDConfig(args[0].sval, args[1].ival, args[2].ival, args[3].ival, args[4].ival);
}

static void andorCCDRegister(void)
{

    iocshRegister(&configAndorCCD, configAndorCCDCallFunc);
}

epicsExportRegistrar(andorCCDRegister);
