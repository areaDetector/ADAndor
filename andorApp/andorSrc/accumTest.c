#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#ifdef _WIN32
#include "ATMCD32D.h"
#else
#include "atmcdLXd.h"
#endif

void checkStatus(int status)
{
  if (status == DRV_SUCCESS) return;
  printf("checkStatus error=%d exiting!\n", status);
  exit(-1);
}

int main(int argc, char *argv[])
{
  int adcChannel=1, minX=0, minY=0, binX=1, binY=1, sizeX=1024, sizeY=1024;
  int triggerMode=0, numExposures=2, numImages=3;
  float mAcquireTime=0.1f, mAccumulatePeriod=1.0f, mAcquirePeriod=4.0f;
  float acquireTimeAct, accumulatePeriodAct, acquirePeriodAct;
  int AAKinetics=3, ATInternal=0;
  time_t startTime, endTime;
  int acquireStatus;

  checkStatus(Initialize("/usr/local/etc/andor"));

  printf("SetTriggerMode(%d)\n", triggerMode);
  checkStatus(SetTriggerMode(ATInternal));
  printf("SetADChannel(%d)\n", adcChannel);
  checkStatus(SetADChannel(adcChannel));
  //Set fastest HS speed.
  printf("SetHSSpeed(0, 0)\n");
  checkStatus(SetHSSpeed(0, 0));
  printf("SetImage(%d,%d,%d,%d,%d,%d)\n", 
    binX, binY, minX+1, minX+sizeX, minY+1, minY+sizeY);
  checkStatus(SetImage(binX, binY, minX+1, minX+sizeX, minY+1, minY+sizeY));

  printf("SetExposureTime(%f)\n", mAcquireTime);
  checkStatus(SetExposureTime(mAcquireTime));

  printf("SetAcquisitionMode(AAKinetics)\n");
  checkStatus(SetAcquisitionMode(AAKinetics));
  printf("SetNumberAccumulations(%d)\n", numExposures);
  checkStatus(SetNumberAccumulations(numExposures));
  printf("SetAccumulationCycleTime(%f)\n", mAccumulatePeriod);
  checkStatus(SetAccumulationCycleTime(mAccumulatePeriod));
  printf("SetNumberKinetics(%d)\n", numImages);
  checkStatus(SetNumberKinetics(numImages));
  printf("SetKineticCycleTime(%f)\n", mAcquirePeriod);
  checkStatus(SetKineticCycleTime(mAcquirePeriod));

  checkStatus(GetAcquisitionTimings(&acquireTimeAct, &accumulatePeriodAct, &acquirePeriodAct));
  printf("GetAcquisitionTimings(exposure=%f, accumulate=%f, kinetic=%f)\n",
    acquireTimeAct, accumulatePeriodAct, acquirePeriodAct);

  time(&startTime);
  printf("StartAcquisition()\n");
  checkStatus(StartAcquisition());
  while (1) {
    printf("GetStatus()\n");
    checkStatus(GetStatus(&acquireStatus));
    if (acquireStatus != DRV_ACQUIRING) break;
    printf("WaitForAcquisition()\n");
    checkStatus(WaitForAcquisition());
    time(&endTime);
    printf("Time since start=%f\n", difftime(endTime, startTime));
  }
  return 0;
}
