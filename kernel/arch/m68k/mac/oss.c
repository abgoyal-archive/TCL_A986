

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/delay.h>
#include <linux/init.h>

#include <asm/bootinfo.h>
#include <asm/macintosh.h>
#include <asm/macints.h>
#include <asm/mac_via.h>
#include <asm/mac_oss.h>

int oss_present;
volatile struct mac_oss *oss;

static irqreturn_t oss_irq(int, void *);
static irqreturn_t oss_nubus_irq(int, void *);

extern irqreturn_t via1_irq(int, void *);


void __init oss_init(void)
{
	int i;

	if (!oss_present) return;

	oss = (struct mac_oss *) OSS_BASE;

	/* Disable all interrupts. Unlike a VIA it looks like we    */
	/* do this by setting the source's interrupt level to zero. */

	for (i = 0; i <= OSS_NUM_SOURCES; i++) {
		oss->irq_level[i] = OSS_IRQLEV_DISABLED;
	}
	/* If we disable VIA1 here, we never really handle it... */
	oss->irq_level[OSS_VIA1] = OSS_IRQLEV_VIA1;
}


void __init oss_register_interrupts(void)
{
	if (request_irq(OSS_IRQLEV_SCSI, oss_irq, IRQ_FLG_LOCK,
			"scsi", (void *) oss))
		pr_err("Couldn't register %s interrupt\n", "scsi");
	if (request_irq(OSS_IRQLEV_NUBUS, oss_nubus_irq, IRQ_FLG_LOCK,
			"nubus", (void *) oss))
		pr_err("Couldn't register %s interrupt\n", "nubus");
	if (request_irq(OSS_IRQLEV_SOUND, oss_irq, IRQ_FLG_LOCK,
			"sound", (void *) oss))
		pr_err("Couldn't register %s interrupt\n", "sound");
	if (request_irq(OSS_IRQLEV_VIA1, via1_irq, IRQ_FLG_LOCK,
			"via1", (void *) via1))
		pr_err("Couldn't register %s interrupt\n", "via1");
}


void __init oss_nubus_init(void)
{
}


static irqreturn_t oss_irq(int irq, void *dev_id)
{
	int events;

	events = oss->irq_pending & (OSS_IP_SOUND|OSS_IP_SCSI);
	if (!events)
		return IRQ_NONE;

#ifdef DEBUG_IRQS
	if ((console_loglevel == 10) && !(events & OSS_IP_SCSI)) {
		printk("oss_irq: irq %d events = 0x%04X\n", irq,
			(int) oss->irq_pending);
	}
#endif
	/* FIXME: how do you clear a pending IRQ?    */

	if (events & OSS_IP_SOUND) {
		oss->irq_pending &= ~OSS_IP_SOUND;
		/* FIXME: call sound handler */
	} else if (events & OSS_IP_SCSI) {
		oss->irq_pending &= ~OSS_IP_SCSI;
		m68k_handle_int(IRQ_MAC_SCSI);
	} else {
		/* FIXME: error check here? */
	}
	return IRQ_HANDLED;
}


static irqreturn_t oss_nubus_irq(int irq, void *dev_id)
{
	int events, irq_bit, i;

	events = oss->irq_pending & OSS_IP_NUBUS;
	if (!events)
		return IRQ_NONE;

#ifdef DEBUG_NUBUS_INT
	if (console_loglevel > 7) {
		printk("oss_nubus_irq: events = 0x%04X\n", events);
	}
#endif
	/* There are only six slots on the OSS, not seven */

	i = 6;
	irq_bit = 0x40;
	do {
		--i;
		irq_bit >>= 1;
		if (events & irq_bit) {
			oss->irq_pending &= ~irq_bit;
			m68k_handle_int(NUBUS_SOURCE_BASE + i);
		}
	} while(events & (irq_bit - 1));
	return IRQ_HANDLED;
}


void oss_irq_enable(int irq) {
#ifdef DEBUG_IRQUSE
	printk("oss_irq_enable(%d)\n", irq);
#endif
	switch(irq) {
		case IRQ_MAC_SCC:
			oss->irq_level[OSS_IOPSCC] = OSS_IRQLEV_IOPSCC;
			break;
		case IRQ_MAC_ADB:
			oss->irq_level[OSS_IOPISM] = OSS_IRQLEV_IOPISM;
			break;
		case IRQ_MAC_SCSI:
			oss->irq_level[OSS_SCSI] = OSS_IRQLEV_SCSI;
			break;
		case IRQ_NUBUS_9:
		case IRQ_NUBUS_A:
		case IRQ_NUBUS_B:
		case IRQ_NUBUS_C:
		case IRQ_NUBUS_D:
		case IRQ_NUBUS_E:
			irq -= NUBUS_SOURCE_BASE;
			oss->irq_level[irq] = OSS_IRQLEV_NUBUS;
			break;
#ifdef DEBUG_IRQUSE
		default:
			printk("%s unknown irq %d\n", __func__, irq);
			break;
#endif
	}
}


void oss_irq_disable(int irq) {
#ifdef DEBUG_IRQUSE
	printk("oss_irq_disable(%d)\n", irq);
#endif
	switch(irq) {
		case IRQ_MAC_SCC:
			oss->irq_level[OSS_IOPSCC] = OSS_IRQLEV_DISABLED;
			break;
		case IRQ_MAC_ADB:
			oss->irq_level[OSS_IOPISM] = OSS_IRQLEV_DISABLED;
			break;
		case IRQ_MAC_SCSI:
			oss->irq_level[OSS_SCSI] = OSS_IRQLEV_DISABLED;
			break;
		case IRQ_NUBUS_9:
		case IRQ_NUBUS_A:
		case IRQ_NUBUS_B:
		case IRQ_NUBUS_C:
		case IRQ_NUBUS_D:
		case IRQ_NUBUS_E:
			irq -= NUBUS_SOURCE_BASE;
			oss->irq_level[irq] = OSS_IRQLEV_DISABLED;
			break;
#ifdef DEBUG_IRQUSE
		default:
			printk("%s unknown irq %d\n", __func__, irq);
			break;
#endif
	}
}


void oss_irq_clear(int irq) {
	/* FIXME: how to do this on OSS? */
	switch(irq) {
		case IRQ_MAC_SCC:
			oss->irq_pending &= ~OSS_IP_IOPSCC;
			break;
		case IRQ_MAC_ADB:
			oss->irq_pending &= ~OSS_IP_IOPISM;
			break;
		case IRQ_MAC_SCSI:
			oss->irq_pending &= ~OSS_IP_SCSI;
			break;
		case IRQ_NUBUS_9:
		case IRQ_NUBUS_A:
		case IRQ_NUBUS_B:
		case IRQ_NUBUS_C:
		case IRQ_NUBUS_D:
		case IRQ_NUBUS_E:
			irq -= NUBUS_SOURCE_BASE;
			oss->irq_pending &= ~(1 << irq);
			break;
	}
}


int oss_irq_pending(int irq)
{
	switch(irq) {
		case IRQ_MAC_SCC:
			return oss->irq_pending & OSS_IP_IOPSCC;
			break;
		case IRQ_MAC_ADB:
			return oss->irq_pending & OSS_IP_IOPISM;
			break;
		case IRQ_MAC_SCSI:
			return oss->irq_pending & OSS_IP_SCSI;
			break;
		case IRQ_NUBUS_9:
		case IRQ_NUBUS_A:
		case IRQ_NUBUS_B:
		case IRQ_NUBUS_C:
		case IRQ_NUBUS_D:
		case IRQ_NUBUS_E:
			irq -= NUBUS_SOURCE_BASE;
			return oss->irq_pending & (1 << irq);
			break;
	}
	return 0;
}
