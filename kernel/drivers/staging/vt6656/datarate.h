
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


#define RATE_1M         0
#define RATE_2M         1
#define RATE_5M         2
#define RATE_11M        3
#define RATE_6M         4
#define RATE_9M         5
#define RATE_12M        6
#define RATE_18M        7
#define RATE_24M        8
#define RATE_36M        9
#define RATE_48M       10
#define RATE_54M       11
#define RATE_AUTO      12
#define MAX_RATE       12

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
     unsigned int                uRateLen
    );

WORD
RATEwGetRateIdx(
     BYTE byRate
    );


BYTE
DATARATEbyGetRateIdx(
     BYTE byRate
    );

#endif /* __DATARATE_H__ */
