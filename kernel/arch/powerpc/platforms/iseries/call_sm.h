
#ifndef _ISERIES_CALL_SM_H
#define _ISERIES_CALL_SM_H


#include <asm/iseries/hv_call_sc.h>
#include <asm/iseries/hv_types.h>

#define HvCallSmGet64BitsOfAccessMap	HvCallSm  + 11

static inline u64 HvCallSm_get64BitsOfAccessMap(HvLpIndex lpIndex,
		u64 indexIntoBitMap)
{
	return HvCall2(HvCallSmGet64BitsOfAccessMap, lpIndex, indexIntoBitMap);
}

#endif /* _ISERIES_CALL_SM_H */
