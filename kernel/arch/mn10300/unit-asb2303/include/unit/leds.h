

#ifndef _ASM_UNIT_LEDS_H
#define _ASM_UNIT_LEDS_H

#include <asm/pio-regs.h>
#include <asm/cpu-regs.h>
#include <asm/exceptions.h>

#define ASB2303_GPIO0DEF	__SYSREG(0xDB000000, u32)
#define ASB2303_7SEGLEDS	__SYSREG(0xDB000008, u32)


/* flip the 7-segment LEDs between "G" and "-" */
#define mn10300_set_gdbleds(ONOFF)			\
do {							\
	ASB2303_7SEGLEDS = (ONOFF) ? 0x85 : 0x7f;	\
} while (0)

/* indicate double-fault by displaying "d" on the LEDs */
#define mn10300_set_dbfleds			\
	mov	0x43,d0			;	\
	movbu	d0,(ASB2303_7SEGLEDS)

#ifndef __ASSEMBLY__
extern void peripheral_leds_display_exception(enum exception_code code);
extern void peripheral_leds_led_chase(void);
extern void debug_to_serial(const char *p, int n);
#endif /* __ASSEMBLY__ */

#endif /* _ASM_UNIT_LEDS_H */
