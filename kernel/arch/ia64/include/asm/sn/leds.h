
#ifndef _ASM_IA64_SN_LEDS_H
#define _ASM_IA64_SN_LEDS_H

#include <asm/sn/addrs.h>
#include <asm/sn/pda.h>
#include <asm/sn/shub_mmr.h>

#define LED0		(LOCAL_MMR_ADDR(SH_REAL_JUNK_BUS_LED0))
#define LED_CPU_SHIFT	16

#define LED_CPU_HEARTBEAT	0x01
#define LED_CPU_ACTIVITY	0x02
#define LED_ALWAYS_SET		0x00


static __inline__ void
set_led_bits(u8 value, u8 mask)
{
	pda->led_state = (pda->led_state & ~mask) | (value & mask);
	*pda->led_address = (short) pda->led_state;
}

#endif /* _ASM_IA64_SN_LEDS_H */
