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
  int minX=0, minY=0, binX=1, binY=1, sizeX, sizeY;
  char model[256];
  float temperature;
  int status;

  checkStatus(Initialize("/usr/local/etc/andor"));
  printf("Intialize(/usr/local/etc/andor) OK\n");
  checkStatus(GetDetector(&sizeX, &sizeY));
  printf("GetDetector() OK, sizeX=%d, sizeY=%d\n", sizeX, sizeY);
  checkStatus(GetHeadModel(model));
  printf("GetHeadModel() OK, model=%s\n", model);
  checkStatus(SetReadMode(4));
  printf("SetReadMode(4) OK\n");
  checkStatus(SetImage(binX, binY, minX+1, minX+sizeX, minY+1, minY+sizeY));
  printf("SetImage OK\n");
  status = GetTemperatureF(&temperature);
  printf("GetTemperature OK, temperature=%f, status=%d\n", temperature, status);
  return 0;
}
