
#ifndef _PLATFORMS_ISERIES_CALL_HPT_H
#define _PLATFORMS_ISERIES_CALL_HPT_H


#include <asm/iseries/hv_call_sc.h>
#include <asm/iseries/hv_types.h>
#include <asm/mmu.h>

#define HvCallHptGetHptAddress		HvCallHpt +  0
#define HvCallHptGetHptPages		HvCallHpt +  1
#define HvCallHptSetPp			HvCallHpt +  5
#define HvCallHptSetSwBits		HvCallHpt +  6
#define HvCallHptUpdate			HvCallHpt +  7
#define HvCallHptInvalidateNoSyncICache	HvCallHpt +  8
#define HvCallHptGet			HvCallHpt + 11
#define HvCallHptFindNextValid		HvCallHpt + 12
#define HvCallHptFindValid		HvCallHpt + 13
#define HvCallHptAddValidate		HvCallHpt + 16
#define HvCallHptInvalidateSetSwBitsGet HvCallHpt + 18


static inline u64 HvCallHpt_getHptAddress(void)
{
	return HvCall0(HvCallHptGetHptAddress);
}

static inline u64 HvCallHpt_getHptPages(void)
{
	return HvCall0(HvCallHptGetHptPages);
}

static inline void HvCallHpt_setPp(u32 hpteIndex, u8 value)
{
	HvCall2(HvCallHptSetPp, hpteIndex, value);
}

static inline void HvCallHpt_setSwBits(u32 hpteIndex, u8 bitson, u8 bitsoff)
{
	HvCall3(HvCallHptSetSwBits, hpteIndex, bitson, bitsoff);
}

static inline void HvCallHpt_invalidateNoSyncICache(u32 hpteIndex)
{
	HvCall1(HvCallHptInvalidateNoSyncICache, hpteIndex);
}

static inline u64 HvCallHpt_invalidateSetSwBitsGet(u32 hpteIndex, u8 bitson,
		u8 bitsoff)
{
	u64 compressedStatus;

	compressedStatus = HvCall4(HvCallHptInvalidateSetSwBitsGet,
			hpteIndex, bitson, bitsoff, 1);
	HvCall1(HvCallHptInvalidateNoSyncICache, hpteIndex);
	return compressedStatus;
}

static inline u64 HvCallHpt_findValid(struct hash_pte *hpte, u64 vpn)
{
	return HvCall3Ret16(HvCallHptFindValid, hpte, vpn, 0, 0);
}

static inline u64 HvCallHpt_findNextValid(struct hash_pte *hpte, u32 hpteIndex,
		u8 bitson, u8 bitsoff)
{
	return HvCall3Ret16(HvCallHptFindNextValid, hpte, hpteIndex,
			bitson, bitsoff);
}

static inline void HvCallHpt_get(struct hash_pte *hpte, u32 hpteIndex)
{
	HvCall2Ret16(HvCallHptGet, hpte, hpteIndex, 0);
}

static inline void HvCallHpt_addValidate(u32 hpteIndex, u32 hBit,
					 struct hash_pte *hpte)
{
	HvCall4(HvCallHptAddValidate, hpteIndex, hBit, hpte->v, hpte->r);
}

#endif /* _PLATFORMS_ISERIES_CALL_HPT_H */
