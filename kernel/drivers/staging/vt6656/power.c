

#include "ttype.h"
#include "mac.h"
#include "device.h"
#include "wmgr.h"
#include "power.h"
#include "wcmd.h"
#include "rxtx.h"
#include "card.h"
#include "control.h"
#include "rndis.h"

/*---------------------  Static Definitions -------------------------*/

/*---------------------  Static Classes  ----------------------------*/

/*---------------------  Static Variables  --------------------------*/
static int          msglevel                =MSG_LEVEL_INFO;
/*---------------------  Static Functions  --------------------------*/

/*---------------------  Export Variables  --------------------------*/

/*---------------------  Export Functions  --------------------------*/


void PSvEnablePowerSaving(void *hDeviceContext,
			  WORD wListenInterval)
{
    PSDevice        pDevice = (PSDevice)hDeviceContext;
    PSMgmtObject    pMgmt = &(pDevice->sMgmtObj);
    WORD            wAID = pMgmt->wCurrAID | BIT14 | BIT15;

    // set period of power up before TBTT
    MACvWriteWord(pDevice, MAC_REG_PWBT, C_PWBT);

    if (pDevice->eOPMode != OP_MODE_ADHOC) {
        // set AID
        MACvWriteWord(pDevice, MAC_REG_AIDATIM, wAID);
    } else {
    	// set ATIM Window
        //MACvWriteATIMW(pDevice->PortOffset, pMgmt->wCurrATIMWindow);
    }

    //Warren:06-18-2004,the sequence must follow PSEN->AUTOSLEEP->GO2DOZE
    // enable power saving hw function
    MACvRegBitsOn(pDevice, MAC_REG_PSCTL, PSCTL_PSEN);
    // Set AutoSleep
    MACvRegBitsOn(pDevice, MAC_REG_PSCFG, PSCFG_AUTOSLEEP);

    //Warren:MUST turn on this once before turn on AUTOSLEEP ,or the AUTOSLEEP doesn't work
    MACvRegBitsOn(pDevice, MAC_REG_PSCTL, PSCTL_GO2DOZE);


    if (wListenInterval >= 2) {

        // clear always listen beacon
        MACvRegBitsOff(pDevice, MAC_REG_PSCTL, PSCTL_ALBCN);
        // first time set listen next beacon
        MACvRegBitsOn(pDevice, MAC_REG_PSCTL, PSCTL_LNBCN);

        pMgmt->wCountToWakeUp = wListenInterval;

    }
    else {

        // always listen beacon
        MACvRegBitsOn(pDevice, MAC_REG_PSCTL, PSCTL_ALBCN);
        pMgmt->wCountToWakeUp = 0;

    }

    pDevice->bEnablePSMode = TRUE;

    if (pDevice->eOPMode == OP_MODE_ADHOC) {
	/* bMgrPrepareBeaconToSend((void *) pDevice, pMgmt); */
    }
    // We don't send null pkt in ad hoc mode since beacon will handle this.
    else if (pDevice->eOPMode == OP_MODE_INFRASTRUCTURE) {
        PSbSendNullPacket(pDevice);
    }
    pDevice->bPWBitOn = TRUE;
    DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO "PS:Power Saving Mode Enable... \n");
    return;
}


void PSvDisablePowerSaving(void *hDeviceContext)
{
    PSDevice        pDevice = (PSDevice)hDeviceContext;
//    PSMgmtObject    pMgmt = &(pDevice->sMgmtObj);


    // disable power saving hw function
    CONTROLnsRequestOut(pDevice,
                        MESSAGE_TYPE_DISABLE_PS,
                        0,
                        0,
                        0,
                        NULL
                        );

    //clear AutoSleep
    MACvRegBitsOff(pDevice, MAC_REG_PSCFG, PSCFG_AUTOSLEEP);

    // set always listen beacon
    MACvRegBitsOn(pDevice, MAC_REG_PSCTL, PSCTL_ALBCN);

    pDevice->bEnablePSMode = FALSE;

    if (pDevice->eOPMode == OP_MODE_INFRASTRUCTURE) {
        PSbSendNullPacket(pDevice);
    }
    pDevice->bPWBitOn = FALSE;
    return;
}


BOOL PSbConsiderPowerDown(void *hDeviceContext,
			  BOOL bCheckRxDMA,
			  BOOL bCheckCountToWakeUp)
{
    PSDevice        pDevice = (PSDevice)hDeviceContext;
    PSMgmtObject    pMgmt = &(pDevice->sMgmtObj);
    BYTE            byData;


    // check if already in Doze mode
    ControlvReadByte(pDevice, MESSAGE_REQUEST_MACREG, MAC_REG_PSCTL, &byData);
    if ( (byData & PSCTL_PS) != 0 )
        return TRUE;;

    if (pMgmt->eCurrMode != WMAC_MODE_IBSS_STA) {
        // check if in TIM wake period
        if (pMgmt->bInTIMWake)
            return FALSE;
    }

    // check scan state
    if (pDevice->bCmdRunning)
        return FALSE;

    //Tx Burst
    if ( pDevice->bPSModeTxBurst )
        return FALSE;

    // Froce PSEN on
    MACvRegBitsOn(pDevice, MAC_REG_PSCTL, PSCTL_PSEN);

    if (pMgmt->eCurrMode != WMAC_MODE_IBSS_STA) {
        if (bCheckCountToWakeUp &&
            (pMgmt->wCountToWakeUp == 0 || pMgmt->wCountToWakeUp == 1)) {
             return FALSE;
        }
    }

    pDevice->bPSRxBeacon = TRUE;
    // no Tx, no Rx isr, now go to Doze
    MACvRegBitsOn(pDevice, MAC_REG_PSCTL, PSCTL_GO2DOZE);

    DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO "Go to Doze ZZZZZZZZZZZZZZZ\n");
    return TRUE;
}


void PSvSendPSPOLL(void *hDeviceContext)
{
    PSDevice            pDevice = (PSDevice)hDeviceContext;
    PSMgmtObject        pMgmt = &(pDevice->sMgmtObj);
    PSTxMgmtPacket      pTxPacket = NULL;


    memset(pMgmt->pbyPSPacketPool, 0, sizeof(STxMgmtPacket) + WLAN_HDR_ADDR2_LEN);
    pTxPacket = (PSTxMgmtPacket)pMgmt->pbyPSPacketPool;
    pTxPacket->p80211Header = (PUWLAN_80211HDR)((PBYTE)pTxPacket + sizeof(STxMgmtPacket));
    pTxPacket->p80211Header->sA2.wFrameCtl = cpu_to_le16(
         (
         WLAN_SET_FC_FTYPE(WLAN_TYPE_CTL) |
         WLAN_SET_FC_FSTYPE(WLAN_FSTYPE_PSPOLL) |
         WLAN_SET_FC_PWRMGT(0)
         ));
    pTxPacket->p80211Header->sA2.wDurationID = pMgmt->wCurrAID | BIT14 | BIT15;
    memcpy(pTxPacket->p80211Header->sA2.abyAddr1, pMgmt->abyCurrBSSID, WLAN_ADDR_LEN);
    memcpy(pTxPacket->p80211Header->sA2.abyAddr2, pMgmt->abyMACAddr, WLAN_ADDR_LEN);
    pTxPacket->cbMPDULen = WLAN_HDR_ADDR2_LEN;
    pTxPacket->cbPayloadLen = 0;
    // send the frame
    if (csMgmt_xmit(pDevice, pTxPacket) != CMD_STATUS_PENDING) {
        DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO "Send PS-Poll packet failed..\n");
    }
    else {
//        DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO "Send PS-Poll packet success..\n");
    };

    return;
}


BOOL PSbSendNullPacket(void *hDeviceContext)
{
    PSDevice            pDevice = (PSDevice)hDeviceContext;
    PSTxMgmtPacket      pTxPacket = NULL;
    PSMgmtObject        pMgmt = &(pDevice->sMgmtObj);



    if (pDevice->bLinkPass == FALSE) {
        return FALSE;
    }

//2007-0115-03<Add>by MikeLiu
#ifdef TxInSleep
     if ((pDevice->bEnablePSMode == FALSE) &&
	  (pDevice->fTxDataInSleep == FALSE)){
        return FALSE;
    }
#else
    if (pDevice->bEnablePSMode == FALSE) {
        return FALSE;
    }
#endif
    memset(pMgmt->pbyPSPacketPool, 0, sizeof(STxMgmtPacket) + WLAN_NULLDATA_FR_MAXLEN);
    pTxPacket = (PSTxMgmtPacket)pMgmt->pbyPSPacketPool;
    pTxPacket->p80211Header = (PUWLAN_80211HDR)((PBYTE)pTxPacket + sizeof(STxMgmtPacket));

    if (pDevice->bEnablePSMode) {

        pTxPacket->p80211Header->sA3.wFrameCtl = cpu_to_le16(
             (
            WLAN_SET_FC_FTYPE(WLAN_TYPE_DATA) |
            WLAN_SET_FC_FSTYPE(WLAN_FSTYPE_NULL) |
            WLAN_SET_FC_PWRMGT(1)
            ));
    }
    else {
        pTxPacket->p80211Header->sA3.wFrameCtl = cpu_to_le16(
             (
            WLAN_SET_FC_FTYPE(WLAN_TYPE_DATA) |
            WLAN_SET_FC_FSTYPE(WLAN_FSTYPE_NULL) |
            WLAN_SET_FC_PWRMGT(0)
            ));
    }

    if(pMgmt->eCurrMode != WMAC_MODE_IBSS_STA) {
        pTxPacket->p80211Header->sA3.wFrameCtl |= cpu_to_le16((WORD)WLAN_SET_FC_TODS(1));
    }

    memcpy(pTxPacket->p80211Header->sA3.abyAddr1, pMgmt->abyCurrBSSID, WLAN_ADDR_LEN);
    memcpy(pTxPacket->p80211Header->sA3.abyAddr2, pMgmt->abyMACAddr, WLAN_ADDR_LEN);
    memcpy(pTxPacket->p80211Header->sA3.abyAddr3, pMgmt->abyCurrBSSID, WLAN_BSSID_LEN);
    pTxPacket->cbMPDULen = WLAN_HDR_ADDR3_LEN;
    pTxPacket->cbPayloadLen = 0;
    // send the frame
    if (csMgmt_xmit(pDevice, pTxPacket) != CMD_STATUS_PENDING) {
        DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO "Send Null Packet failed !\n");
        return FALSE;
    }
    else {
//            DBG_PRT(MSG_LEVEL_DEBUG, KERN_INFO "Send Null Packet success....\n");
    }


    return TRUE ;
}


BOOL PSbIsNextTBTTWakeUp(void *hDeviceContext)
{

    PSDevice         pDevice = (PSDevice)hDeviceContext;
    PSMgmtObject        pMgmt = &(pDevice->sMgmtObj);
    BOOL                bWakeUp = FALSE;

    if (pMgmt->wListenInterval >= 2) {
        if (pMgmt->wCountToWakeUp == 0) {
            pMgmt->wCountToWakeUp = pMgmt->wListenInterval;
        }

        pMgmt->wCountToWakeUp --;

        if (pMgmt->wCountToWakeUp == 1) {

            // Turn on wake up to listen next beacon
            MACvRegBitsOn(pDevice, MAC_REG_PSCTL, PSCTL_LNBCN);
            pDevice->bPSRxBeacon = FALSE;
            bWakeUp = TRUE;

        } else if ( !pDevice->bPSRxBeacon ) {
            //Listen until RxBeacon
            MACvRegBitsOn(pDevice, MAC_REG_PSCTL, PSCTL_LNBCN);
        }

    }

    return bWakeUp;
}

