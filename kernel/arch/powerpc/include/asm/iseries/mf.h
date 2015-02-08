
#ifndef _ASM_POWERPC_ISERIES_MF_H
#define _ASM_POWERPC_ISERIES_MF_H

#include <linux/types.h>

#include <asm/iseries/hv_types.h>
#include <asm/iseries/hv_call_event.h>

struct rtc_time;

typedef void (*MFCompleteHandler)(void *clientToken, int returnCode);

extern void mf_allocate_lp_events(HvLpIndex targetLp, HvLpEvent_Type type,
		unsigned size, unsigned amount, MFCompleteHandler hdlr,
		void *userToken);
extern void mf_deallocate_lp_events(HvLpIndex targetLp, HvLpEvent_Type type,
		unsigned count, MFCompleteHandler hdlr, void *userToken);

extern void mf_power_off(void);
extern void mf_reboot(char *cmd);

extern void mf_display_src(u32 word);
extern void mf_display_progress(u16 value);

extern void mf_init(void);

#endif /* _ASM_POWERPC_ISERIES_MF_H */
