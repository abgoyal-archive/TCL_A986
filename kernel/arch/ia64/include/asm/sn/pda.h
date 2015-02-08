
#ifndef _ASM_IA64_SN_PDA_H
#define _ASM_IA64_SN_PDA_H

#include <linux/cache.h>
#include <asm/percpu.h>
#include <asm/system.h>



typedef struct pda_s {

	/*
	 * Support for SN LEDs
	 */
	volatile short	*led_address;
	u8		led_state;
	u8		hb_state;	/* supports blinking heartbeat leds */
	unsigned int	hb_count;

	unsigned int	idle_flag;
	
	volatile unsigned long *bedrock_rev_id;
	volatile unsigned long *pio_write_status_addr;
	unsigned long pio_write_status_val;
	volatile unsigned long *pio_shub_war_cam_addr;

	unsigned long	sn_in_service_ivecs[4];
	int		sn_lb_int_war_ticks;
	int		sn_last_irq;
	int		sn_first_irq;
} pda_t;


#define CACHE_ALIGN(x)	(((x) + SMP_CACHE_BYTES-1) & ~(SMP_CACHE_BYTES-1))

DECLARE_PER_CPU(struct pda_s, pda_percpu);

#define pda		(&__ia64_per_cpu_var(pda_percpu))

#define pdacpu(cpu)	(&per_cpu(pda_percpu, cpu))

#endif /* _ASM_IA64_SN_PDA_H */
