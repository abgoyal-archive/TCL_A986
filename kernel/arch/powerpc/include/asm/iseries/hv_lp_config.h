
#ifndef _ASM_POWERPC_ISERIES_HV_LP_CONFIG_H
#define _ASM_POWERPC_ISERIES_HV_LP_CONFIG_H


#include <asm/iseries/hv_call_sc.h>
#include <asm/iseries/hv_types.h>

enum {
	HvCallCfg_Cur	= 0,
	HvCallCfg_Init	= 1,
	HvCallCfg_Max	= 2,
	HvCallCfg_Min	= 3
};

#define HvCallCfgGetSystemPhysicalProcessors		HvCallCfg +  6
#define HvCallCfgGetPhysicalProcessors			HvCallCfg +  7
#define HvCallCfgGetMsChunks				HvCallCfg +  9
#define HvCallCfgGetSharedPoolIndex			HvCallCfg + 20
#define HvCallCfgGetSharedProcUnits			HvCallCfg + 21
#define HvCallCfgGetNumProcsInSharedPool		HvCallCfg + 22
#define HvCallCfgGetVirtualLanIndexMap			HvCallCfg + 30
#define HvCallCfgGetHostingLpIndex			HvCallCfg + 32

extern HvLpIndex HvLpConfig_getLpIndex_outline(void);
extern HvLpIndex HvLpConfig_getLpIndex(void);
extern HvLpIndex HvLpConfig_getPrimaryLpIndex(void);

static inline u64 HvLpConfig_getMsChunks(void)
{
	return HvCall2(HvCallCfgGetMsChunks, HvLpConfig_getLpIndex(),
			HvCallCfg_Cur);
}

static inline u64 HvLpConfig_getSystemPhysicalProcessors(void)
{
	return HvCall0(HvCallCfgGetSystemPhysicalProcessors);
}

static inline u64 HvLpConfig_getNumProcsInSharedPool(HvLpSharedPoolIndex sPI)
{
	return (u16)HvCall1(HvCallCfgGetNumProcsInSharedPool, sPI);
}

static inline u64 HvLpConfig_getPhysicalProcessors(void)
{
	return HvCall2(HvCallCfgGetPhysicalProcessors, HvLpConfig_getLpIndex(),
			HvCallCfg_Cur);
}

static inline HvLpSharedPoolIndex HvLpConfig_getSharedPoolIndex(void)
{
	return HvCall1(HvCallCfgGetSharedPoolIndex, HvLpConfig_getLpIndex());
}

static inline u64 HvLpConfig_getSharedProcUnits(void)
{
	return HvCall2(HvCallCfgGetSharedProcUnits, HvLpConfig_getLpIndex(),
			HvCallCfg_Cur);
}

static inline u64 HvLpConfig_getMaxSharedProcUnits(void)
{
	return HvCall2(HvCallCfgGetSharedProcUnits, HvLpConfig_getLpIndex(),
			HvCallCfg_Max);
}

static inline u64 HvLpConfig_getMaxPhysicalProcessors(void)
{
	return HvCall2(HvCallCfgGetPhysicalProcessors, HvLpConfig_getLpIndex(),
			HvCallCfg_Max);
}

static inline HvLpVirtualLanIndexMap HvLpConfig_getVirtualLanIndexMapForLp(
		HvLpIndex lp)
{
	/*
	 * This is a new function in V5R1 so calls to this on older
	 * hypervisors will return -1
	 */
	u64 retVal = HvCall1(HvCallCfgGetVirtualLanIndexMap, lp);
	if (retVal == -1)
		retVal = 0;
	return retVal;
}

static inline HvLpVirtualLanIndexMap HvLpConfig_getVirtualLanIndexMap(void)
{
	return HvLpConfig_getVirtualLanIndexMapForLp(
			HvLpConfig_getLpIndex_outline());
}

static inline int HvLpConfig_doLpsCommunicateOnVirtualLan(HvLpIndex lp1,
		HvLpIndex lp2)
{
	HvLpVirtualLanIndexMap virtualLanIndexMap1 =
		HvLpConfig_getVirtualLanIndexMapForLp(lp1);
	HvLpVirtualLanIndexMap virtualLanIndexMap2 =
		HvLpConfig_getVirtualLanIndexMapForLp(lp2);
	return ((virtualLanIndexMap1 & virtualLanIndexMap2) != 0);
}

static inline HvLpIndex HvLpConfig_getHostingLpIndex(HvLpIndex lp)
{
	return HvCall1(HvCallCfgGetHostingLpIndex, lp);
}

#endif /* _ASM_POWERPC_ISERIES_HV_LP_CONFIG_H */
