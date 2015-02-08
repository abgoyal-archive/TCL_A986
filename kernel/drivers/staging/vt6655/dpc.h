

#ifndef __DPC_H__
#define __DPC_H__

#include "ttype.h"
#include "device.h"
#include "wcmd.h"

/*---------------------  Export Definitions -------------------------*/

/*---------------------  Export Classes  ----------------------------*/

/*---------------------  Export Variables  --------------------------*/

/*---------------------  Export Functions  --------------------------*/

BOOL
device_receive_frame (
    PSDevice pDevice,
    PSRxDesc pCurrRD
    );

void	MngWorkItem(void *Context);

#endif // __RXTX_H__



