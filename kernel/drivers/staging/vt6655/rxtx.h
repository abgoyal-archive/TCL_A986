

#ifndef __RXTX_H__
#define __RXTX_H__

#include "ttype.h"
#include "device.h"
#include "wcmd.h"

/*---------------------  Export Definitions -------------------------*/

/*---------------------  Export Variables  --------------------------*/

/*---------------------  Export Functions  --------------------------*/



void
vGenerateMACHeader (
    PSDevice         pDevice,
    PBYTE            pbyBufferAddr,
    WORD             wDuration,
    PSEthernetHeader psEthHeader,
    BOOL             bNeedEncrypt,
    WORD             wFragType,
    UINT             uDMAIdx,
    UINT             uFragIdx
    );


UINT
cbGetFragCount(
    PSDevice         pDevice,
    PSKeyItem        pTransmitKey,
    UINT             cbFrameBodySize,
    PSEthernetHeader psEthHeader
    );


void
vGenerateFIFOHeader (
    PSDevice         pDevice,
    BYTE             byPktTyp,
    PBYTE            pbyTxBufferAddr,
    BOOL             bNeedEncrypt,
    UINT             cbPayloadSize,
    UINT             uDMAIdx,
    PSTxDesc         pHeadTD,
    PSEthernetHeader psEthHeader,
    PBYTE            pPacket,
    PSKeyItem        pTransmitKey,
    UINT             uNodeIndex,
    PUINT            puMACfragNum,
    PUINT            pcbHeaderSize
    );


void vDMA0_tx_80211(PSDevice  pDevice, struct sk_buff *skb, PBYTE pbMPDU, UINT cbMPDULen);
CMD_STATUS csMgmt_xmit(PSDevice pDevice, PSTxMgmtPacket pPacket);
CMD_STATUS csBeacon_xmit(PSDevice pDevice, PSTxMgmtPacket pPacket);

#endif // __RXTX_H__
