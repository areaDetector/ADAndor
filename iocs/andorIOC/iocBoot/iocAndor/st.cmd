< envPaths
errlogInit(20000)

dbLoadDatabase("$(TOP)/dbd/andorCCDApp.dbd")
andorCCDApp_registerRecordDeviceDriver(pdbbase) 

epicsEnvSet("PREFIX", "13ANDOR1:")
epicsEnvSet("PORT",   "ANDOR")
epicsEnvSet("QSIZE",  "20")
epicsEnvSet("XSIZE",  "2048")
epicsEnvSet("YSIZE",  "2048")
epicsEnvSet("NCHANS", "2048")
# The maximum number of frames buffered in the NDPluginCircularBuff plugin
epicsEnvSet("CBUFFS", "500")
# The search path for database files
epicsEnvSet("EPICS_DB_INCLUDE_PATH", "$(ADCORE)/db")

# andorCCDConfig(const char *portName, const char *installPath, int shamrockID,
#                int maxBuffers, size_t maxMemory, int priority, int stackSize)
#andorCCDConfig("$(PORT)", "/usr/local/etc/andor/", 0, 0, 0, 0 ,0)
andorCCDConfig("$(PORT)", "", 0, 0, 0, 0, 0)

dbLoadRecords("$(ADANDOR)/db/andorCCD.template",   "P=$(PREFIX),R=cam1:,PORT=$(PORT),ADDR=0,TIMEOUT=1")

# Comment out the following lines if there is no Shamrock spectrograph
#shamrockConfig(const char *portName, int shamrockId, const char *iniPath, int priority, int stackSize)
shamrockConfig("SR1", 0, "", 0, 0)
dbLoadRecords("$(ADANDOR)/db/shamrock.template",   "P=$(PREFIX),R=sham1:,PORT=SR1,TIMEOUT=1,PIXELS=1024")

# Create a standard arrays plugin
NDStdArraysConfigure("Image1", 5, 0, "$(PORT)", 0, 0)
# Make NELEMENTS in the following be a little bigger than 2048*2048
# Use the following command for 32-bit images.  This is needed for 32-bit detectors or for 16-bit detectors in acccumulate mode if it would overflow 16 bits
#dbLoadRecords("$(ADCORE)/db/NDStdArrays.template", "P=$(PREFIX),R=image1:,PORT=Image1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Int32,FTVL=LONG,NELEMENTS=4200000")
# Use the following command for 16-bit images.  This can be used for 16-bit detector as long as accumulate mode would not result in 16-bit overflow
dbLoadRecords("$(ADCORE)/db/NDStdArrays.template", "P=$(PREFIX),R=image1:,PORT=Image1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Int16,FTVL=SHORT,NELEMENTS=4200000")

# Load all other plugins using commonPlugins.cmd
< $(ADCORE)/iocBoot/commonPlugins.cmd
set_requestfile_path("$(ADANDOR)/andorApp/Db")

#asynSetTraceMask("$(PORT)",0,3)
#asynSetTraceIOMask("$(PORT)",0,4)

iocInit()

# save things every thirty seconds
create_monitor_set("auto_settings.req", 30,"P=$(PREFIX)")
#asynSetTraceMask($(PORT), 0, 255)
