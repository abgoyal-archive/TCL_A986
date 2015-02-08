

#ifndef __WPACTL_H__
#define __WPACTL_H__

#include "device.h"
#ifdef WPA_SUPPLICANT_DRIVER_WEXT_SUPPORT
#include "iowpa.h"
#endif

/*---------------------  Export Definitions -------------------------*/


//WPA related

typedef enum { WPA_ALG_NONE, WPA_ALG_WEP, WPA_ALG_TKIP, WPA_ALG_CCMP } wpa_alg;
typedef enum { CIPHER_NONE, CIPHER_WEP40, CIPHER_TKIP, CIPHER_CCMP,
	       CIPHER_WEP104 } wpa_cipher;
typedef enum { KEY_MGMT_802_1X, KEY_MGMT_CCKM,KEY_MGMT_PSK, KEY_MGMT_NONE,
	       KEY_MGMT_802_1X_NO_WPA, KEY_MGMT_WPA_NONE } wpa_key_mgmt;

#define AUTH_ALG_OPEN_SYSTEM	0x01
#define AUTH_ALG_SHARED_KEY	0x02
#define AUTH_ALG_LEAP		0x04

#define GENERIC_INFO_ELEM 0xdd
#define RSN_INFO_ELEM 0x30



typedef ULONGLONG   NDIS_802_11_KEY_RSC;

/*---------------------  Export Classes  ----------------------------*/

/*---------------------  Export Variables  --------------------------*/

/*---------------------  Export Functions  --------------------------*/

int wpa_set_wpadev(PSDevice pDevice, int val);
int wpa_ioctl(PSDevice pDevice, struct iw_point *p);
int wpa_set_keys(PSDevice pDevice, void *ctx, BOOL  fcpfkernel);

#endif // __WPACL_H__



