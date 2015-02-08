

#ifndef __FIRMWARE_H__
#define __FIRMWARE_H__

#include "ttype.h"
#include "device.h"

/*---------------------  Export Definitions -------------------------*/

/*---------------------  Export Classes  ----------------------------*/

/*---------------------  Export Variables  --------------------------*/

/*---------------------  Export Functions  --------------------------*/

BOOL
FIRMWAREbDownload(
     PSDevice pDevice
    );

BOOL
FIRMWAREbBrach2Sram(
     PSDevice pDevice
    );

BOOL
FIRMWAREbCheckVersion(
     PSDevice pDevice
    );

#endif /* __FIRMWARE_H__ */
