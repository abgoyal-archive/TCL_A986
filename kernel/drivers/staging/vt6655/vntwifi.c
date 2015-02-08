

#include "vntwifi.h"
#include "IEEE11h.h"
#include "country.h"
#include "device.h"
#include "wmgr.h"
#include "datarate.h"

//#define	PLICE_DEBUG

/*---------------------  Static Definitions -------------------------*/
//static int          msglevel                =MSG_LEVEL_DEBUG;
//static int          msglevel                =MSG_LEVEL_INFO;

/*---------------------  Static Classes  ----------------------------*/

/*---------------------  Static Variables  --------------------------*/

/*---------------------  Static Functions  --------------------------*/

/*---------------------  Export Variables  --------------------------*/

/*---------------------  Export Functions  --------------------------*/

void
VNTWIFIvSetOPMode (
    void *pMgmtHandle,
    WMAC_CONFIG_MODE eOPMode
    )
{
    PSMgmtObject        pMgmt = (PSMgmtObject)pMgmtHandle;

    pMgmt->eConfigMode = eOPMode;
}


void
VNTWIFIvSetIBSSParameter (
    void *pMgmtHandle,
    WORD  wBeaconPeriod,
    WORD  wATIMWindow,
    UINT  uChannel
    )
{
    PSMgmtObject        pMgmt = (PSMgmtObject)pMgmtHandle;

    pMgmt->wIBSSBeaconPeriod = wBeaconPeriod;
    pMgmt->wIBSSATIMWindow = wATIMWindow;
    pMgmt->uIBSSChannel = uChannel;
}

PWLAN_IE_SSID
VNTWIFIpGetCurrentSSID (
    void *pMgmtHandle
    )
{
    PSMgmtObject        pMgmt = (PSMgmtObject)pMgmtHandle;
    return((PWLAN_IE_SSID) pMgmt->abyCurrSSID);
}

UINT
VNTWIFIpGetCurrentChannel (
    void *pMgmtHandle
    )
{
    PSMgmtObject        pMgmt = (PSMgmtObject)pMgmtHandle;
    if (pMgmtHandle != NULL) {
        return (pMgmt->uCurrChannel);
    }
    return 0;
}

WORD
VNTWIFIwGetAssocID (
    void *pMgmtHandle
    )
{
    PSMgmtObject        pMgmt = (PSMgmtObject)pMgmtHandle;
    return(pMgmt->wCurrAID);
}



BYTE
VNTWIFIbyGetMaxSupportRate (
    PWLAN_IE_SUPP_RATES pSupportRateIEs,
    PWLAN_IE_SUPP_RATES pExtSupportRateIEs
    )
{
    BYTE    byMaxSupportRate = RATE_1M;
    BYTE    bySupportRate = RATE_1M;
    UINT    ii = 0;

    if (pSupportRateIEs) {
        for (ii = 0; ii < pSupportRateIEs->len; ii++) {
            bySupportRate = DATARATEbyGetRateIdx(pSupportRateIEs->abyRates[ii]);
            if (bySupportRate > byMaxSupportRate) {
                byMaxSupportRate = bySupportRate;
            }
        }
    }
    if (pExtSupportRateIEs) {
        for (ii = 0; ii < pExtSupportRateIEs->len; ii++) {
            bySupportRate = DATARATEbyGetRateIdx(pExtSupportRateIEs->abyRates[ii]);
            if (bySupportRate > byMaxSupportRate) {
                byMaxSupportRate = bySupportRate;
            }
        }
    }

    return byMaxSupportRate;
}

BYTE
VNTWIFIbyGetACKTxRate (
    BYTE byRxDataRate,
    PWLAN_IE_SUPP_RATES pSupportRateIEs,
    PWLAN_IE_SUPP_RATES pExtSupportRateIEs
    )
{
    BYTE    byMaxAckRate;
    BYTE    byBasicRate;
    UINT    ii;

    if (byRxDataRate <= RATE_11M) {
        byMaxAckRate = RATE_1M;
    } else  {
        // 24M is mandatory for 802.11a and 802.11g
        byMaxAckRate = RATE_24M;
    }
    if (pSupportRateIEs) {
        for (ii = 0; ii < pSupportRateIEs->len; ii++) {
            if (pSupportRateIEs->abyRates[ii] & 0x80) {
                byBasicRate = DATARATEbyGetRateIdx(pSupportRateIEs->abyRates[ii]);
                if ((byBasicRate <= byRxDataRate) &&
                    (byBasicRate > byMaxAckRate))  {
                    byMaxAckRate = byBasicRate;
                }
            }
        }
    }
    if (pExtSupportRateIEs) {
        for (ii = 0; ii < pExtSupportRateIEs->len; ii++) {
            if (pExtSupportRateIEs->abyRates[ii] & 0x80) {
                byBasicRate = DATARATEbyGetRateIdx(pExtSupportRateIEs->abyRates[ii]);
                if ((byBasicRate <= byRxDataRate) &&
                    (byBasicRate > byMaxAckRate))  {
                    byMaxAckRate = byBasicRate;
                }
            }
        }
    }

    return byMaxAckRate;
}

void
VNTWIFIvSetAuthenticationMode (
    void *pMgmtHandle,
    WMAC_AUTHENTICATION_MODE eAuthMode
    )
{
    PSMgmtObject        pMgmt = (PSMgmtObject)pMgmtHandle;

    pMgmt->eAuthenMode = eAuthMode;
    if ((eAuthMode == WMAC_AUTH_SHAREKEY) ||
        (eAuthMode == WMAC_AUTH_AUTO)) {
        pMgmt->bShareKeyAlgorithm = TRUE;
    } else {
        pMgmt->bShareKeyAlgorithm = FALSE;
    }
}

void
VNTWIFIvSetEncryptionMode (
    void *pMgmtHandle,
    WMAC_ENCRYPTION_MODE eEncryptionMode
    )
{
    PSMgmtObject        pMgmt = (PSMgmtObject)pMgmtHandle;

    pMgmt->eEncryptionMode = eEncryptionMode;
    if ((eEncryptionMode == WMAC_ENCRYPTION_WEPEnabled) ||
        (eEncryptionMode == WMAC_ENCRYPTION_TKIPEnabled) ||
        (eEncryptionMode == WMAC_ENCRYPTION_AESEnabled) ) {
        pMgmt->bPrivacyInvoked = TRUE;
    } else {
        pMgmt->bPrivacyInvoked = FALSE;
    }
}



BOOL
VNTWIFIbConfigPhyMode (
    void *pMgmtHandle,
    CARD_PHY_TYPE ePhyType
    )
{
    PSMgmtObject        pMgmt = (PSMgmtObject)pMgmtHandle;

    if ((ePhyType != PHY_TYPE_AUTO) &&
        (ePhyType != pMgmt->eCurrentPHYMode)) {
        if (CARDbSetPhyParameter(pMgmt->pAdapter, ePhyType, 0, 0, NULL, NULL)==TRUE) {
            pMgmt->eCurrentPHYMode = ePhyType;
        } else {
            return(FALSE);
        }
    }
    pMgmt->eConfigPHYMode = ePhyType;
    return(TRUE);
}


void
VNTWIFIbGetConfigPhyMode (
    void *pMgmtHandle,
    void *pePhyType
    )
{
    PSMgmtObject        pMgmt = (PSMgmtObject)pMgmtHandle;

    if ((pMgmt != NULL) && (pePhyType != NULL)) {
        *(PCARD_PHY_TYPE)pePhyType = pMgmt->eConfigPHYMode;
    }
}




void
VNTWIFIvQueryBSSList (
    void *pMgmtHandle,
    PUINT   puBSSCount,
    void **pvFirstBSS
    )
{
    UINT            ii = 0;
    PSMgmtObject    pMgmt = (PSMgmtObject)pMgmtHandle;
    PKnownBSS       pBSS = NULL;
    UINT            uCount = 0;

    *pvFirstBSS = NULL;

    for (ii = 0; ii < MAX_BSS_NUM; ii++) {
        pBSS = &(pMgmt->sBSSList[ii]);
        if (!pBSS->bActive) {
            continue;
        }
        if (*pvFirstBSS == NULL) {
            *pvFirstBSS = &(pMgmt->sBSSList[ii]);
        }
        uCount++;
    }
    *puBSSCount = uCount;
}




void
VNTWIFIvGetNextBSS (
    void *pMgmtHandle,
    void *pvCurrentBSS,
    void **pvNextBSS
    )
{
    PKnownBSS       pBSS = (PKnownBSS) pvCurrentBSS;
    PSMgmtObject    pMgmt = (PSMgmtObject)pMgmtHandle;

    *pvNextBSS = NULL;

    while (*pvNextBSS == NULL) {
        pBSS++;
        if (pBSS > &(pMgmt->sBSSList[MAX_BSS_NUM])) {
            return;
        }
        if (pBSS->bActive == TRUE) {
            *pvNextBSS = pBSS;
            return;
        }
    }
}





void
VNTWIFIvUpdateNodeTxCounter(
    void *pMgmtHandle,
    PBYTE    pbyDestAddress,
    BOOL     bTxOk,
    WORD     wRate,
    PBYTE    pbyTxFailCount
    )
{
    PSMgmtObject    pMgmt = (PSMgmtObject)pMgmtHandle;
    UINT            uNodeIndex = 0;
    UINT            ii;

    if ((pMgmt->eCurrMode == WMAC_MODE_IBSS_STA) ||
        (pMgmt->eCurrMode == WMAC_MODE_ESS_AP)) {
        if (BSSDBbIsSTAInNodeDB(pMgmt, pbyDestAddress, &uNodeIndex) == FALSE) {
            return;
        }
    }
    pMgmt->sNodeDBTable[uNodeIndex].uTxAttempts++;
    if (bTxOk == TRUE) {
        // transmit success, TxAttempts at least plus one
        pMgmt->sNodeDBTable[uNodeIndex].uTxOk[MAX_RATE]++;
        pMgmt->sNodeDBTable[uNodeIndex].uTxOk[wRate]++;
    } else {
        pMgmt->sNodeDBTable[uNodeIndex].uTxFailures++;
    }
    pMgmt->sNodeDBTable[uNodeIndex].uTxRetry += pbyTxFailCount[MAX_RATE];
    for(ii=0;ii<MAX_RATE;ii++) {
        pMgmt->sNodeDBTable[uNodeIndex].uTxFail[ii] += pbyTxFailCount[ii];
    }
    return;
}


void
VNTWIFIvGetTxRate(
    void *pMgmtHandle,
    PBYTE    pbyDestAddress,
    PWORD   pwTxDataRate,
    PBYTE   pbyACKRate,
    PBYTE   pbyCCKBasicRate,
    PBYTE   pbyOFDMBasicRate
    )
{
    PSMgmtObject        pMgmt = (PSMgmtObject)pMgmtHandle;
    UINT                uNodeIndex = 0;
    WORD                wTxDataRate = RATE_1M;
    BYTE                byACKRate = RATE_1M;
    BYTE                byCCKBasicRate = RATE_1M;
    BYTE                byOFDMBasicRate = RATE_24M;
    PWLAN_IE_SUPP_RATES pSupportRateIEs = NULL;
    PWLAN_IE_SUPP_RATES pExtSupportRateIEs = NULL;


    if ((pMgmt->eCurrMode == WMAC_MODE_IBSS_STA) ||
        (pMgmt->eCurrMode == WMAC_MODE_ESS_AP)) {
        // Adhoc Tx rate decided from node DB
        if(BSSDBbIsSTAInNodeDB(pMgmt, pbyDestAddress, &uNodeIndex)) {
            wTxDataRate = (pMgmt->sNodeDBTable[uNodeIndex].wTxDataRate);
            pSupportRateIEs = (PWLAN_IE_SUPP_RATES) (pMgmt->sNodeDBTable[uNodeIndex].abyCurrSuppRates);
            pExtSupportRateIEs = (PWLAN_IE_SUPP_RATES) (pMgmt->sNodeDBTable[uNodeIndex].abyCurrExtSuppRates);
        } else {
            if (pMgmt->eCurrentPHYMode != PHY_TYPE_11A) {
                wTxDataRate = RATE_2M;
            } else {
                wTxDataRate = RATE_24M;
            }
            pSupportRateIEs = (PWLAN_IE_SUPP_RATES) pMgmt->abyCurrSuppRates;
            pExtSupportRateIEs = (PWLAN_IE_SUPP_RATES) pMgmt->abyCurrExtSuppRates;
        }
    } else { // Infrastructure: rate decided from AP Node, index = 0

		wTxDataRate = (pMgmt->sNodeDBTable[0].wTxDataRate);
#ifdef	PLICE_DEBUG
		printk("GetTxRate:AP MAC is %02x:%02x:%02x:%02x:%02x:%02x,TxRate is %d\n",
				pMgmt->sNodeDBTable[0].abyMACAddr[0],pMgmt->sNodeDBTable[0].abyMACAddr[1],
				pMgmt->sNodeDBTable[0].abyMACAddr[2],pMgmt->sNodeDBTable[0].abyMACAddr[3],
				pMgmt->sNodeDBTable[0].abyMACAddr[4],pMgmt->sNodeDBTable[0].abyMACAddr[5],wTxDataRate);
#endif


        pSupportRateIEs = (PWLAN_IE_SUPP_RATES) pMgmt->abyCurrSuppRates;
        pExtSupportRateIEs = (PWLAN_IE_SUPP_RATES) pMgmt->abyCurrExtSuppRates;
    }
    byACKRate = VNTWIFIbyGetACKTxRate(  (BYTE) wTxDataRate,
                                        pSupportRateIEs,
                                        pExtSupportRateIEs
                                        );
    if (byACKRate > (BYTE) wTxDataRate) {
        byACKRate = (BYTE) wTxDataRate;
    }
    byCCKBasicRate = VNTWIFIbyGetACKTxRate( RATE_11M,
                                            pSupportRateIEs,
                                            pExtSupportRateIEs
                                            );
    byOFDMBasicRate = VNTWIFIbyGetACKTxRate(RATE_54M,
                                            pSupportRateIEs,
                                            pExtSupportRateIEs
                                            );
    *pwTxDataRate = wTxDataRate;
    *pbyACKRate = byACKRate;
    *pbyCCKBasicRate = byCCKBasicRate;
    *pbyOFDMBasicRate = byOFDMBasicRate;
    return;
}

BYTE
VNTWIFIbyGetKeyCypher(
    void *pMgmtHandle,
    BOOL     bGroupKey
    )
{
    PSMgmtObject    pMgmt = (PSMgmtObject)pMgmtHandle;

    if (bGroupKey == TRUE) {
        return (pMgmt->byCSSGK);
    } else {
        return (pMgmt->byCSSPK);
    }
}





BOOL
VNTWIFIbSetPMKIDCache (
    void *pMgmtObject,
    ULONG ulCount,
    void *pPMKIDInfo
    )
{
    PSMgmtObject    pMgmt = (PSMgmtObject) pMgmtObject;

    if (ulCount > MAX_PMKID_CACHE) {
        return (FALSE);
    }
    pMgmt->gsPMKIDCache.BSSIDInfoCount = ulCount;
    memcpy(pMgmt->gsPMKIDCache.BSSIDInfo, pPMKIDInfo, (ulCount*sizeof(PMKIDInfo)));
    return (TRUE);
}



WORD
VNTWIFIwGetMaxSupportRate(
    void *pMgmtObject
    )
{
    WORD wRate = RATE_54M;
    PSMgmtObject    pMgmt = (PSMgmtObject) pMgmtObject;

    for(wRate = RATE_54M; wRate > RATE_1M; wRate--) {
        if (pMgmt->sNodeDBTable[0].wSuppRate & (1<<wRate)) {
            return (wRate);
        }
    }
    if (pMgmt->eCurrentPHYMode == PHY_TYPE_11A) {
        return (RATE_6M);
    } else {
        return (RATE_1M);
    }
}


void
VNTWIFIvSet11h (
    void *pMgmtObject,
    BOOL  b11hEnable
    )
{
    PSMgmtObject    pMgmt = (PSMgmtObject) pMgmtObject;

    pMgmt->b11hEnable = b11hEnable;
}

BOOL
VNTWIFIbMeasureReport(
    void *pMgmtObject,
    BOOL  bEndOfReport,
    void *pvMeasureEID,
    BYTE  byReportMode,
    BYTE  byBasicMap,
    BYTE  byCCAFraction,
    PBYTE pbyRPIs
    )
{
    PSMgmtObject    pMgmt = (PSMgmtObject) pMgmtObject;
    PBYTE           pbyCurrentEID = (PBYTE) (pMgmt->pCurrMeasureEIDRep);

    //spin_lock_irq(&pDevice->lock);
    if ((pvMeasureEID != NULL) &&
        (pMgmt->uLengthOfRepEIDs < (WLAN_A3FR_MAXLEN - sizeof(MEASEURE_REP) - sizeof(WLAN_80211HDR_A3) - 3))
        ) {
        pMgmt->pCurrMeasureEIDRep->byElementID = WLAN_EID_MEASURE_REP;
        pMgmt->pCurrMeasureEIDRep->len = 3;
        pMgmt->pCurrMeasureEIDRep->byToken = ((PWLAN_IE_MEASURE_REQ) pvMeasureEID)->byToken;
        pMgmt->pCurrMeasureEIDRep->byMode = byReportMode;
        pMgmt->pCurrMeasureEIDRep->byType = ((PWLAN_IE_MEASURE_REQ) pvMeasureEID)->byType;
        switch (pMgmt->pCurrMeasureEIDRep->byType) {
            case MEASURE_TYPE_BASIC :
                pMgmt->pCurrMeasureEIDRep->len += sizeof(MEASEURE_REP_BASIC);
                memcpy(   &(pMgmt->pCurrMeasureEIDRep->sRep.sBasic),
                            &(((PWLAN_IE_MEASURE_REQ) pvMeasureEID)->sReq),
                            sizeof(MEASEURE_REQ));
                pMgmt->pCurrMeasureEIDRep->sRep.sBasic.byMap = byBasicMap;
                break;
            case MEASURE_TYPE_CCA :
                pMgmt->pCurrMeasureEIDRep->len += sizeof(MEASEURE_REP_CCA);
                memcpy(   &(pMgmt->pCurrMeasureEIDRep->sRep.sCCA),
                            &(((PWLAN_IE_MEASURE_REQ) pvMeasureEID)->sReq),
                            sizeof(MEASEURE_REQ));
                pMgmt->pCurrMeasureEIDRep->sRep.sCCA.byCCABusyFraction = byCCAFraction;
                break;
            case MEASURE_TYPE_RPI :
                pMgmt->pCurrMeasureEIDRep->len += sizeof(MEASEURE_REP_RPI);
                memcpy(   &(pMgmt->pCurrMeasureEIDRep->sRep.sRPI),
                            &(((PWLAN_IE_MEASURE_REQ) pvMeasureEID)->sReq),
                            sizeof(MEASEURE_REQ));
                memcpy(pMgmt->pCurrMeasureEIDRep->sRep.sRPI.abyRPIdensity, pbyRPIs, 8);
                break;
            default :
                break;
        }
        pbyCurrentEID += (2 + pMgmt->pCurrMeasureEIDRep->len);
        pMgmt->uLengthOfRepEIDs += (2 + pMgmt->pCurrMeasureEIDRep->len);
        pMgmt->pCurrMeasureEIDRep = (PWLAN_IE_MEASURE_REP) pbyCurrentEID;
    }
    if (bEndOfReport == TRUE) {
        IEEE11hbMSRRepTx(pMgmt);
    }
    //spin_unlock_irq(&pDevice->lock);
    return (TRUE);
}


BOOL
VNTWIFIbChannelSwitch(
    void *pMgmtObject,
    BYTE  byNewChannel
    )
{
    PSMgmtObject    pMgmt = (PSMgmtObject) pMgmtObject;

    //spin_lock_irq(&pDevice->lock);
    pMgmt->uCurrChannel = byNewChannel;
    pMgmt->bSwitchChannel = FALSE;
    //spin_unlock_irq(&pDevice->lock);
    return TRUE;
}


