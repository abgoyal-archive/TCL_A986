
#ifndef __AP_H__
#define __AP_H__

/* ap_wpa.c */
void WpaStateMachineInit(struct rt_rtmp_adapter *pAd,
			 struct rt_state_machine *Sm, OUT STATE_MACHINE_FUNC Trans[]);

#ifdef RTMP_MAC_USB
void BeaconUpdateExec(void *SystemSpecific1,
		      void *FunctionContext,
		      void *SystemSpecific2, void *SystemSpecific3);
#endif /* RTMP_MAC_USB // */

void RTMPSetPiggyBack(struct rt_rtmp_adapter *pAd, IN BOOLEAN bPiggyBack);

void MacTableReset(struct rt_rtmp_adapter *pAd);

struct rt_mac_table_entry *MacTableInsertEntry(struct rt_rtmp_adapter *pAd,
				     u8 *pAddr,
				     u8 apidx, IN BOOLEAN CleanAll);

BOOLEAN MacTableDeleteEntry(struct rt_rtmp_adapter *pAd,
			    u16 wcid, u8 *pAddr);

struct rt_mac_table_entry *MacTableLookup(struct rt_rtmp_adapter *pAd, u8 *pAddr);

#endif /* __AP_H__ */
