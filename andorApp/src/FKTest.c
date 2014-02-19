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
  int triggerMode=0, numImages=5, FKOffset, FKRows=64, FKMode=4;
  float mAcquireTime=0.1f;
  float acquireTimeAct;
  at_32 *pArray;
  int AAFastKinetics=4, ATInternal=0;
  time_t startTime, endTime;
  int i;
  int firstImage, lastImage;
  at_32 validFirst, validLast;
  char fileName[256];
  char *palFilePath = "./GREY.PAL";
  
  pArray = (at_32 *)malloc(sizeX * sizeY * sizeof(int));

  printf("Initialize(\"\")\n");
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

  printf("SetAcquisitionMode(AAFastKinetics)\n");
  checkStatus(SetAcquisitionMode(AAFastKinetics));
  FKOffset = sizeY - FKRows - minY;
  printf("SetFastKineticsEx(%d,%d,%f,%d,%d,%d,%d)\n", 
    FKRows, numImages, mAcquireTime, FKMode, binX, binY, FKOffset);
  checkStatus(SetFastKineticsEx(FKRows, numImages, mAcquireTime, FKMode, binX, binY, FKOffset));

  checkStatus(GetFKExposureTime(&acquireTimeAct));

  printf("GetFKExposureTime(exposure=%f)\n", acquireTimeAct);

  time(&startTime);
  printf("StartAcquisition()\n");
  checkStatus(StartAcquisition());
  printf("WaitForAcquisition()\n");
  checkStatus(WaitForAcquisition());
  time(&endTime);
  printf("Time since start=%f\n", difftime(endTime, startTime));
  checkStatus(GetNumberNewImages(&firstImage, &lastImage));
  printf("First image=%d, last image=%d\n", firstImage, lastImage);
  
  for (i=firstImage; i<=lastImage; i++) {
    printf("Processing image %d\n", i);
    sprintf(fileName, "tiff_test_%d.tif", i);
    printf("  SaveAsTiffEx(%s, %s, %d, 1, 1)\n", fileName, palFilePath, i);
    checkStatus(SaveAsTiffEx(fileName, palFilePath, i, 1, 1));
    printf("  GetImages(%d, %d, %p, %d, %p, %p)\n", i, i, pArray, sizeX*FKRows, &validFirst, &validLast);
    checkStatus(GetImages(i, i, (at_32*)pArray, sizeX*FKRows, &validFirst, &validLast));
  }
  return 0;
}
