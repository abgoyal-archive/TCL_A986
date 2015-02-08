

#ifndef __USBPIPE_H__
#define __USBPIPE_H__

#include "ttype.h"
#include "device.h"

/*---------------------  Export Definitions -------------------------*/

/*---------------------  Export Classes  ----------------------------*/

/*---------------------  Export Variables  --------------------------*/

/*---------------------  Export Functions  --------------------------*/

NTSTATUS
PIPEnsControlOut(
     PSDevice     pDevice,
     BYTE         byRequest,
     WORD         wValue,
     WORD         wIndex,
     WORD         wLength,
     PBYTE        pbyBuffer
    );



NTSTATUS
PIPEnsControlOutAsyn(
     PSDevice     pDevice,
     BYTE         byRequest,
     WORD         wValue,
     WORD         wIndex,
     WORD         wLength,
     PBYTE        pbyBuffer
    );

NTSTATUS
PIPEnsControlIn(
     PSDevice     pDevice,
     BYTE         byRequest,
     WORD         wValue,
     WORD         wIndex,
     WORD         wLength,
       PBYTE   pbyBuffer
    );




NTSTATUS
PIPEnsInterruptRead(
     PSDevice pDevice
    );

NTSTATUS
PIPEnsBulkInUsbRead(
     PSDevice pDevice,
     PRCB     pRCB
    );

NTSTATUS
PIPEnsSendBulkOut(
      PSDevice pDevice,
      PUSB_SEND_CONTEXT pContext
    );

#endif /* __USBPIPE_H__ */
