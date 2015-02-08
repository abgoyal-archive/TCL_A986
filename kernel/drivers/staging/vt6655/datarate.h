
#ifndef __DATARATE_H__
#define __DATARATE_H__

/*---------------------  Export Definitions -------------------------*/

#define FALLBACK_PKT_COLLECT_TR_H  50   // pkts
#define FALLBACK_PKT_COLLECT_TR_L  10   // pkts
#define FALLBACK_POLL_SECOND       5    // 5 sec
#define FALLBACK_RECOVER_SECOND    30   // 30 sec
#define FALLBACK_THRESHOLD         15   // percent
#define UPGRADE_THRESHOLD          5    // percent
#define UPGRADE_CNT_THRD           3    // times
#define RETRY_TIMES_THRD_H         2    // times
#define RETRY_TIMES_THRD_L         1    // times


/*---------------------  Export Classes  ----------------------------*/

/*---------------------  Export Variables  --------------------------*/


/*---------------------  Export Types  ------------------------------*/


/*---------------------  Export Functions  --------------------------*/



void
RATEvParseMaxRate(
    void *pDeviceHandler,
    PWLAN_IE_SUPP_RATES pItemRates,
    PWLAN_IE_SUPP_RATES pItemExtRates,
    BOOL bUpdateBasicRate,
    PWORD pwMaxBasicRate,
    PWORD pwMaxSuppRate,
    PWORD pwSuppRate,
    PBYTE pbyTopCCKRate,
    PBYTE pbyTopOFDMRate
    );

void
RATEvTxRateFallBack(
    void *pDeviceHandler,
    PKnownNodeDB psNodeDBTable
    );

BYTE
RATEuSetIE(
    PWLAN_IE_SUPP_RATES pSrcRates,
    PWLAN_IE_SUPP_RATES pDstRates,
    UINT                uRateLen
    );

WORD
wGetRateIdx(
    BYTE byRate
    );


BYTE
DATARATEbyGetRateIdx(
    BYTE byRate
    );


#endif //__DATARATE_H__
