
#ifndef APPLE80211_H
#define APPLE80211_H


typedef struct __WirelessRef *WirelessRef;
typedef SInt32 WirelessError;
#define errWirelessNoError 0

typedef struct WirelessInfo {
	UInt16 link_qual;
	UInt16 comms_qual;
	UInt16 signal;
	UInt16 noise;
	UInt16 port_stat;
	UInt16 client_mode;
	UInt16 res1;
	UInt16 power;
	UInt16 res2;
	UInt8 bssID[6];
	UInt8 ssid[34];
} WirelessInfo;

typedef struct WirelessInfo2 {
	/* TODO - these are probably not in correct order or complete */
	WirelessInfo info1;
	UInt8 macAddress[6];
} WirelessInfo2;

typedef struct WirelessNetworkInfo {
	UInt16 channel;
	UInt16 noise;
	UInt16 signal;
	UInt8 bssid[6];
	UInt16 beacon_int;
	UInt16 capability;
	UInt16 ssid_len;
	UInt8 ssid[32];
} WirelessNetworkInfo;

typedef int wirelessKeyType; /* TODO */

int WirelessIsAvailable(void);
WirelessError WirelessAttach(WirelessRef *ref, UInt32 res);
WirelessError WirelessDetach(WirelessRef ref);
WirelessError WirelessPrivate(WirelessRef ref, void *in_ptr, int in_bytes,
			      void *out_ptr, int out_bytes);
WirelessError WirelessSetEnabled(WirelessRef ref, UInt8 enabled);
WirelessError WirelessGetEnabled(WirelessRef ref, UInt8 *enabled);
WirelessError WirelessSetPower(WirelessRef ref, UInt8 power);
WirelessError WirelessGetPower(WirelessRef ref, UInt8 *power);
WirelessError WirelessGetInfo(WirelessRef ref, WirelessInfo *info);
WirelessError WirelessGetInfo2(WirelessRef ref, WirelessInfo2 *info);
WirelessError WirelessScan(WirelessRef ref, CFArrayRef *results,
			   UInt32 strip_dups);
WirelessError WirelessScanSplit(WirelessRef ref, CFArrayRef *ap_results,
				CFArrayRef *ibss_results, UInt32 strip_dups);
WirelessError WirelessDirectedScan(WirelessRef ref, CFArrayRef *results,
				   UInt32 strip_dups, CFStringRef ssid);
WirelessError WirelessDirectedScan2(WirelessRef ref, CFDataRef ssid,
				    UInt32 strip_dups, CFArrayRef *results);
WirelessError WirelessJoin(WirelessRef ref, CFStringRef ssid);
WirelessError WirelessJoinWEP(WirelessRef ref, CFStringRef ssid,
			      CFStringRef passwd);
WirelessError WirelessJoin8021x(WirelessRef ref, CFStringRef ssid);
WirelessError WirelessSetKey(WirelessRef ref, wirelessKeyType type,
			     int key_idx, int key_len,
			     const unsigned char *key);
WirelessError WirelessSetWPAKey(WirelessRef ref, wirelessKeyType type,
				int key_len, const unsigned char *key);
WirelessError WirelessAssociate(WirelessRef ref, int type, CFDataRef ssid,
				CFStringRef key);
WirelessError WirelessAssociate2(WirelessRef ref, CFDictionaryRef scan_res,
				 CFStringRef key);
WirelessError WirelessDisassociate(WirelessRef ref);

CFDictionaryRef WirelessSafeDirectedScanCopy(WirelessRef ref, CFDataRef ssid);

CFDictionaryRef WirelessGetAssociationInfo(WirelessRef ref);

WirelessError WirelessConfigure(WirelessRef ref);

CFDictionaryRef WirelessGetInfoASP(void);

CFDictionaryRef WirelessCopyInterfaceDict(void);

#endif /* APPLE80211_H */
