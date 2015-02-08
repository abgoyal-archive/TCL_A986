

#ifndef __POWER_H__
#define __POWER_H__


/*---------------------  Export Definitions -------------------------*/
#define     C_PWBT                   1000      // micro sec. power up before TBTT
#define     PS_FAST_INTERVAL         1         // Fast power saving listen interval
#define     PS_MAX_INTERVAL          4         // MAX power saving listen interval

/*---------------------  Export Classes  ----------------------------*/

/*---------------------  Export Variables  --------------------------*/


/*---------------------  Export Types  ------------------------------*/


/*---------------------  Export Functions  --------------------------*/

/*  PSDevice pDevice */
/*  PSDevice hDeviceContext */

BOOL PSbConsiderPowerDown(void *hDeviceContext,
			  BOOL bCheckRxDMA,
			  BOOL bCheckCountToWakeUp);

void PSvDisablePowerSaving(void *hDeviceContext);
void PSvEnablePowerSaving(void *hDeviceContext, WORD wListenInterval);
void PSvSendPSPOLL(void *hDeviceContext);
BOOL PSbSendNullPacket(void *hDeviceContext);
BOOL PSbIsNextTBTTWakeUp(void *hDeviceContext);

#endif /* __POWER_H__ */
