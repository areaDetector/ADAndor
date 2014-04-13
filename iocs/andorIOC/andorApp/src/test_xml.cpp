#include "tinyxml.h"
#include <stdio.h>

#define NUM_CHANNELS 1024
#define MAX_CHARS_PER_CHANNEL 20

int main(int argc, const char **argv)
{
  FILE *fp;
  float calibration[NUM_CHANNELS];
  char tempString[MAX_CHARS_PER_CHANNEL];
  char calibrationString[NUM_CHANNELS * MAX_CHARS_PER_CHANNEL];
  TiXmlNode *node;
  TiXmlText *text;
  TiXmlDocument *doc;
  int i;
  
  doc = new TiXmlDocument("SPETemplate.xml");
  doc->LoadFile();
  fp = fopen("test2.xml", "wb");
  fprintf(fp, "This is line 1\n");
  fprintf(fp, "This is line 2\n");
  fprintf(fp, "Now comes the XML\n");
  
  for (i=0; i<NUM_CHANNELS; i++) {
    if (i > 0) strcat(calibrationString, ",");
    calibration[i] = 400. + .01*i;
    sprintf(tempString, "%.6f", calibration[i]);
    strcat(calibrationString, tempString);
  }
  node = doc->FirstChild("SpeFormat");
  node = node->FirstChild("Calibrations");
  node = node->FirstChild("WavelengthMapping");
  node = node->FirstChildElement("Wavelength");
  node = node->FirstChild();
  text = node->ToText();
  text->SetValue(calibrationString);
  doc->SaveFile(fp);
  fclose(fp);
}
