

#ifndef __WPA2_H__
#define __WPA2_H__

#include "ttype.h"
#include "80211mgr.h"
#include "80211hdr.h"
#include "bssdb.h"

/*---------------------  Export Definitions -------------------------*/
#define MAX_PMKID_CACHE         16

typedef struct tagsPMKIDInfo {
    BYTE    abyBSSID[6];
    BYTE    abyPMKID[16];
} PMKIDInfo, *PPMKIDInfo;

typedef struct tagSPMKIDCache {
    unsigned long       BSSIDInfoCount;
    PMKIDInfo   BSSIDInfo[MAX_PMKID_CACHE];
} SPMKIDCache, *PSPMKIDCache;


/*---------------------  Export Classes  ----------------------------*/

/*---------------------  Export Variables  --------------------------*/

/*---------------------  Export Types  ------------------------------*/

/*---------------------  Export Functions  --------------------------*/

void
WPA2_ClearRSN (
     PKnownBSS        pBSSNode
    );

void
WPA2vParseRSN (
     PKnownBSS        pBSSNode,
     PWLAN_IE_RSN     pRSN
    );

unsigned int
WPA2uSetIEs(
	void *pMgmtHandle,
     PWLAN_IE_RSN pRSNIEs
    );

#endif /* __WPA2_H__ */
