

#ifndef __WPA_H__
#define __WPA_H__

#include "ttype.h"
#include "80211hdr.h"

/*---------------------  Export Definitions -------------------------*/

#define WPA_NONE            0
#define WPA_WEP40           1
#define WPA_TKIP            2
#define WPA_AESWRAP         3
#define WPA_AESCCMP         4
#define WPA_WEP104          5
#define WPA_AUTH_IEEE802_1X 1
#define WPA_AUTH_PSK        2

#define WPA_GROUPFLAG       0x02
#define WPA_REPLAYBITSSHIFT 2
#define WPA_REPLAYBITS      0x03

/*---------------------  Export Classes  ----------------------------*/

/*---------------------  Export Variables  --------------------------*/

/*---------------------  Export Types  ------------------------------*/


/*---------------------  Export Functions  --------------------------*/

void
WPA_ClearRSN(
     PKnownBSS        pBSSList
    );

void
WPA_ParseRSN(
     PKnownBSS        pBSSList,
     PWLAN_IE_RSN_EXT pRSN
    );

BOOL
WPA_SearchRSN(
    BYTE                byCmd,
    BYTE                byEncrypt,
     PKnownBSS        pBSSList
    );

BOOL
WPAb_Is_RSN(
     PWLAN_IE_RSN_EXT pRSN
    );

#endif /* __WPA_H__ */
