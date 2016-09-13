/* 
 * File:   DataLoggingConfig.h
 * Author: Brian
 *
 * Created on September 7, 2016, 1:26 PM
 */
#include "ff.h"

#ifndef DATALOGGINGCONFIG_H
#define	DATALOGGINGCONFIG_H

#ifdef	__cplusplus
extern "C" {
#endif
#define SDClusterSize   4096
#define SDSectorSize    512
#define SampleSize      15      //bytes: Sensor #, x1, x2, y1, y2, z1, z2, Year, Month, Day, Hour, Minute, Second, Milisecond1, Milisecond2

FRESULT  DiskStatus;

#ifdef	__cplusplus
}
#endif

#endif	/* DATALOGGINGCONFIG_H */

