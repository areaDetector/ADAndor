
/**
 * EPICS driver for the Andor CCD.
 *
 * @author Matthew Pearson
 * @date June 2009
 *
 * @version 1.0
 *
 */

#ifndef DRV_ANDORCCD_H
#define DRV_ANDORCCD_H

#ifdef __cplusplus
extern "C" {
#endif

int andorCCDConfig(const char *portName, int maxBuffers, size_t maxMemory, int maxSizeX, int maxSizeY);

#ifdef __cplusplus
}
#endif
#endif
