

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/pci.h>
#include <linux/init.h>

#include <asm/ptrace.h>
#include <asm/system.h>
#include <asm/dma.h>
#include <asm/irq.h>
#include <asm/mmu_context.h>
#include <asm/io.h>
#include <asm/pgtable.h>
#include <asm/core_mcpcia.h>
#include <asm/tlbflush.h>

#include "proto.h"
#include "irq_impl.h"
#include "pci_impl.h"
#include "machvec_impl.h"




/* Note mask bit is true for ENABLED irqs.  */

static unsigned int hose_irq_masks[4] = {
	0xff0000, 0xfe0000, 0xff0000, 0xff0000
};
static unsigned int cached_irq_masks[4];
DEFINE_SPINLOCK(rawhide_irq_lock);

static inline void
rawhide_update_irq_hw(int hose, int mask)
{
	*(vuip)MCPCIA_INT_MASK0(MCPCIA_HOSE2MID(hose)) = mask;
	mb();
	*(vuip)MCPCIA_INT_MASK0(MCPCIA_HOSE2MID(hose));
}

#define hose_exists(h) \
  (((h) < MCPCIA_MAX_HOSES) && (cached_irq_masks[(h)] != 0))

static inline void 
rawhide_enable_irq(unsigned int irq)
{
	unsigned int mask, hose;

	irq -= 16;
	hose = irq / 24;
	if (!hose_exists(hose)) /* if hose non-existent, exit */
		return;

	irq -= hose * 24;
	mask = 1 << irq;

	spin_lock(&rawhide_irq_lock);
	mask |= cached_irq_masks[hose];
	cached_irq_masks[hose] = mask;
	rawhide_update_irq_hw(hose, mask);
	spin_unlock(&rawhide_irq_lock);
}

static void 
rawhide_disable_irq(unsigned int irq)
{
	unsigned int mask, hose;

	irq -= 16;
	hose = irq / 24;
	if (!hose_exists(hose)) /* if hose non-existent, exit */
		return;

	irq -= hose * 24;
	mask = ~(1 << irq) | hose_irq_masks[hose];

	spin_lock(&rawhide_irq_lock);
	mask &= cached_irq_masks[hose];
	cached_irq_masks[hose] = mask;
	rawhide_update_irq_hw(hose, mask);
	spin_unlock(&rawhide_irq_lock);
}

static void
rawhide_mask_and_ack_irq(unsigned int irq)
{
	unsigned int mask, mask1, hose;

	irq -= 16;
	hose = irq / 24;
	if (!hose_exists(hose)) /* if hose non-existent, exit */
		return;

	irq -= hose * 24;
	mask1 = 1 << irq;
	mask = ~mask1 | hose_irq_masks[hose];

	spin_lock(&rawhide_irq_lock);

	mask &= cached_irq_masks[hose];
	cached_irq_masks[hose] = mask;
	rawhide_update_irq_hw(hose, mask);

	/* Clear the interrupt.  */
	*(vuip)MCPCIA_INT_REQ(MCPCIA_HOSE2MID(hose)) = mask1;

	spin_unlock(&rawhide_irq_lock);
}

static unsigned int
rawhide_startup_irq(unsigned int irq)
{
	rawhide_enable_irq(irq);
	return 0;
}

static void
rawhide_end_irq(unsigned int irq)
{
	if (!(irq_desc[irq].status & (IRQ_DISABLED|IRQ_INPROGRESS)))
		rawhide_enable_irq(irq);
}

static struct irq_chip rawhide_irq_type = {
	.name		= "RAWHIDE",
	.startup	= rawhide_startup_irq,
	.shutdown	= rawhide_disable_irq,
	.enable		= rawhide_enable_irq,
	.disable	= rawhide_disable_irq,
	.ack		= rawhide_mask_and_ack_irq,
	.end		= rawhide_end_irq,
};

static void 
rawhide_srm_device_interrupt(unsigned long vector)
{
	int irq;

	irq = (vector - 0x800) >> 4;

        /*
         * The RAWHIDE SRM console reports PCI interrupts with a vector
	 * 0x80 *higher* than one might expect, as PCI IRQ 0 (ie bit 0)
	 * shows up as IRQ 24, etc, etc. We adjust it down by 8 to have
	 * it line up with the actual bit numbers from the REQ registers,
	 * which is how we manage the interrupts/mask. Sigh...
	 *
	 * Also, PCI #1 interrupts are offset some more... :-(
         */

	if (irq == 52) {
		/* SCSI on PCI1 is special.  */
		irq = 72;
	}

	/* Adjust by which hose it is from.  */
	irq -= ((irq + 16) >> 2) & 0x38;

	handle_irq(irq);
}

static void __init
rawhide_init_irq(void)
{
	struct pci_controller *hose;
	long i;

	mcpcia_init_hoses();

	/* Clear them all; only hoses that exist will be non-zero. */
	for (i = 0; i < MCPCIA_MAX_HOSES; i++) cached_irq_masks[i] = 0;

	for (hose = hose_head; hose; hose = hose->next) {
		unsigned int h = hose->index;
		unsigned int mask = hose_irq_masks[h];

		cached_irq_masks[h] = mask;
		*(vuip)MCPCIA_INT_MASK0(MCPCIA_HOSE2MID(h)) = mask;
		*(vuip)MCPCIA_INT_MASK1(MCPCIA_HOSE2MID(h)) = 0;
	}

	for (i = 16; i < 128; ++i) {
		irq_desc[i].status = IRQ_DISABLED | IRQ_LEVEL;
		irq_desc[i].chip = &rawhide_irq_type;
	}

	init_i8259a_irqs();
	common_init_isa_dma();
}


static int __init
rawhide_map_irq(struct pci_dev *dev, u8 slot, u8 pin)
{
	static char irq_tab[5][5] __initdata = {
		/*INT    INTA   INTB   INTC   INTD */
		{ 16+16, 16+16, 16+16, 16+16, 16+16}, /* IdSel 1 SCSI PCI 1 */
		{ 16+ 0, 16+ 0, 16+ 1, 16+ 2, 16+ 3}, /* IdSel 2 slot 2 */
		{ 16+ 4, 16+ 4, 16+ 5, 16+ 6, 16+ 7}, /* IdSel 3 slot 3 */
		{ 16+ 8, 16+ 8, 16+ 9, 16+10, 16+11}, /* IdSel 4 slot 4 */
		{ 16+12, 16+12, 16+13, 16+14, 16+15}  /* IdSel 5 slot 5 */
	};
	const long min_idsel = 1, max_idsel = 5, irqs_per_slot = 5;

	struct pci_controller *hose = dev->sysdata;
	int irq = COMMON_TABLE_LOOKUP;
	if (irq >= 0)
		irq += 24 * hose->index;
	return irq;
}



struct alpha_machine_vector rawhide_mv __initmv = {
	.vector_name		= "Rawhide",
	DO_EV5_MMU,
	DO_DEFAULT_RTC,
	DO_MCPCIA_IO,
	.machine_check		= mcpcia_machine_check,
	.max_isa_dma_address	= ALPHA_MAX_ISA_DMA_ADDRESS,
	.min_io_address		= DEFAULT_IO_BASE,
	.min_mem_address	= MCPCIA_DEFAULT_MEM_BASE,
	.pci_dac_offset		= MCPCIA_DAC_OFFSET,

	.nr_irqs		= 128,
	.device_interrupt	= rawhide_srm_device_interrupt,

	.init_arch		= mcpcia_init_arch,
	.init_irq		= rawhide_init_irq,
	.init_rtc		= common_init_rtc,
	.init_pci		= common_init_pci,
	.kill_arch		= NULL,
	.pci_map_irq		= rawhide_map_irq,
	.pci_swizzle		= common_swizzle,
};
ALIAS_MV(rawhide)
