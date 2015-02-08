

#include <linux/init.h>
#include <linux/sched.h>
#include <linux/ioport.h>
#include <linux/mm.h>
#include <linux/console.h>
#include <linux/mc146818rtc.h>
#include <linux/delay.h>

#include <asm/cpu.h>
#include <asm/bootinfo.h>
#include <asm/irq.h>
#include <asm/mipsregs.h>
#include <asm/reboot.h>
#include <asm/pgtable.h>

#include <glb.h>

void __init board_setup(void)
{
	unsigned long configpr;

	configpr = read_c0_config7();
	configpr |= (1<<19); /* enable tlb */
	write_c0_config7(configpr);
}
