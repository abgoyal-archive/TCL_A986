

#ifndef __DPC_H__
#define __DPC_H__

#include "ttype.h"
#include "device.h"
#include "wcmd.h"

/*---------------------  Export Definitions -------------------------*/

/*---------------------  Export Classes  ----------------------------*/

/*---------------------  Export Variables  --------------------------*/

/*---------------------  Export Functions  --------------------------*/

void RXvWorkItem(void *Context);

void RXvMngWorkItem(void *Context);

void
RXvFreeRCB(
     PRCB pRCB,
     BOOL bReAllocSkb
    );

BOOL
RXbBulkInProcessData(
     PSDevice         pDevice,
     PRCB             pRCB,
     unsigned long            BytesToIndicate
    );

#endif /* __RXTX_H__ */
