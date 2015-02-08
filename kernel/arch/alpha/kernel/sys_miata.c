

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/reboot.h>

#include <asm/ptrace.h>
#include <asm/system.h>
#include <asm/dma.h>
#include <asm/irq.h>
#include <asm/mmu_context.h>
#include <asm/io.h>
#include <asm/pgtable.h>
#include <asm/core_cia.h>
#include <asm/tlbflush.h>

#include "proto.h"
#include "irq_impl.h"
#include "pci_impl.h"
#include "machvec_impl.h"


static void 
miata_srm_device_interrupt(unsigned long vector)
{
	int irq;

	irq = (vector - 0x800) >> 4;

	/*
	 * I really hate to do this, but the MIATA SRM console ignores the
	 *  low 8 bits in the interrupt summary register, and reports the
	 *  vector 0x80 *lower* than I expected from the bit numbering in
	 *  the documentation.
	 * This was done because the low 8 summary bits really aren't used
	 *  for reporting any interrupts (the PCI-ISA bridge, bit 7, isn't
	 *  used for this purpose, as PIC interrupts are delivered as the
	 *  vectors 0x800-0x8f0).
	 * But I really don't want to change the fixup code for allocation
	 *  of IRQs, nor the alpha_irq_mask maintenance stuff, both of which
	 *  look nice and clean now.
	 * So, here's this grotty hack... :-(
	 */
	if (irq >= 16)
		irq = irq + 8;

	handle_irq(irq);
}

static void __init
miata_init_irq(void)
{
	if (alpha_using_srm)
		alpha_mv.device_interrupt = miata_srm_device_interrupt;

#if 0
	/* These break on MiataGL so we'll try not to do it at all.  */
	*(vulp)PYXIS_INT_HILO = 0x000000B2UL; mb();	/* ISA/NMI HI */
	*(vulp)PYXIS_RT_COUNT = 0UL; mb();		/* clear count */
#endif

	init_i8259a_irqs();

	/* Not interested in the bogus interrupts (3,10), Fan Fault (0),
           NMI (1), or EIDE (9).

	   We also disable the risers (4,5), since we don't know how to
	   route the interrupts behind the bridge.  */
	init_pyxis_irqs(0x63b0000);

	common_init_isa_dma();
	setup_irq(16+2, &halt_switch_irqaction);	/* SRM only? */
	setup_irq(16+6, &timer_cascade_irqaction);
}



static int __init
miata_map_irq(struct pci_dev *dev, u8 slot, u8 pin)
{
        static char irq_tab[18][5] __initdata = {
		/*INT    INTA   INTB   INTC   INTD */
		{16+ 8, 16+ 8, 16+ 8, 16+ 8, 16+ 8},  /* IdSel 14,  DC21142 */
		{   -1,    -1,    -1,    -1,    -1},  /* IdSel 15,  EIDE    */
		{   -1,    -1,    -1,    -1,    -1},  /* IdSel 16,  none    */
		{   -1,    -1,    -1,    -1,    -1},  /* IdSel 17,  none    */
		{   -1,    -1,    -1,    -1,    -1},  /* IdSel 18,  PCI-ISA */
		{   -1,    -1,    -1,    -1,    -1},  /* IdSel 19,  PCI-PCI */
		{   -1,    -1,    -1,    -1,    -1},  /* IdSel 20,  none    */
		{   -1,    -1,    -1,    -1,    -1},  /* IdSel 21,  none    */
		{16+12, 16+12, 16+13, 16+14, 16+15},  /* IdSel 22,  slot 4  */
		{16+16, 16+16, 16+17, 16+18, 16+19},  /* IdSel 23,  slot 5  */
		/* the next 7 are actually on PCI bus 1, across the bridge */
		{16+11, 16+11, 16+11, 16+11, 16+11},  /* IdSel 24,  QLISP/GL*/
		{   -1,    -1,    -1,    -1,    -1},  /* IdSel 25,  none    */
		{   -1,    -1,    -1,    -1,    -1},  /* IdSel 26,  none    */
		{   -1,    -1,    -1,    -1,    -1},  /* IdSel 27,  none    */
		{16+20, 16+20, 16+21, 16+22, 16+23},  /* IdSel 28,  slot 1  */
		{16+24, 16+24, 16+25, 16+26, 16+27},  /* IdSel 29,  slot 2  */
		{16+28, 16+28, 16+29, 16+30, 16+31},  /* IdSel 30,  slot 3  */
		/* This bridge is on the main bus of the later orig MIATA */
		{   -1,    -1,    -1,    -1,    -1},  /* IdSel 31,  PCI-PCI */
        };
	const long min_idsel = 3, max_idsel = 20, irqs_per_slot = 5;
	
	/* the USB function of the 82c693 has it's interrupt connected to 
           the 2nd 8259 controller. So we have to check for it first. */

	if((slot == 7) && (PCI_FUNC(dev->devfn) == 3)) {
		u8 irq=0;
		struct pci_dev *pdev = pci_get_slot(dev->bus, dev->devfn & ~7);
		if(pdev == NULL || pci_read_config_byte(pdev, 0x40,&irq) != PCIBIOS_SUCCESSFUL) {
			pci_dev_put(pdev);
			return -1;
		}
		else	{
			pci_dev_put(pdev);
			return irq;
		}
	}

	return COMMON_TABLE_LOOKUP;
}

static u8 __init
miata_swizzle(struct pci_dev *dev, u8 *pinp)
{
	int slot, pin = *pinp;

	if (dev->bus->number == 0) {
		slot = PCI_SLOT(dev->devfn);
	}		
	/* Check for the built-in bridge.  */
	else if ((PCI_SLOT(dev->bus->self->devfn) == 8) ||
		 (PCI_SLOT(dev->bus->self->devfn) == 20)) {
		slot = PCI_SLOT(dev->devfn) + 9;
	}
	else 
	{
		/* Must be a card-based bridge.  */
		do {
			if ((PCI_SLOT(dev->bus->self->devfn) == 8) ||
			    (PCI_SLOT(dev->bus->self->devfn) == 20)) {
				slot = PCI_SLOT(dev->devfn) + 9;
				break;
			}
			pin = pci_swizzle_interrupt_pin(dev, pin);

			/* Move up the chain of bridges.  */
			dev = dev->bus->self;
			/* Slot of the next bridge.  */
			slot = PCI_SLOT(dev->devfn);
		} while (dev->bus->self);
	}
	*pinp = pin;
	return slot;
}

static void __init
miata_init_pci(void)
{
	cia_init_pci();
	SMC669_Init(0); /* it might be a GL (fails harmlessly if not) */
	es1888_init();
}

static void
miata_kill_arch(int mode)
{
	cia_kill_arch(mode);

#ifndef ALPHA_RESTORE_SRM_SETUP
	switch(mode) {
	case LINUX_REBOOT_CMD_RESTART:
		/* Who said DEC engineers have no sense of humor? ;-)  */ 
		if (alpha_using_srm) {
			*(vuip) PYXIS_RESET = 0x0000dead; 
			mb(); 
		}
		break;
	case LINUX_REBOOT_CMD_HALT:
		break;
	case LINUX_REBOOT_CMD_POWER_OFF:
		break;
	}

	halt();
#endif
}



struct alpha_machine_vector miata_mv __initmv = {
	.vector_name		= "Miata",
	DO_EV5_MMU,
	DO_DEFAULT_RTC,
	DO_PYXIS_IO,
	.machine_check		= cia_machine_check,
	.max_isa_dma_address	= ALPHA_MAX_ISA_DMA_ADDRESS,
	.min_io_address		= DEFAULT_IO_BASE,
	.min_mem_address	= DEFAULT_MEM_BASE,
	.pci_dac_offset		= PYXIS_DAC_OFFSET,

	.nr_irqs		= 48,
	.device_interrupt	= pyxis_device_interrupt,

	.init_arch		= pyxis_init_arch,
	.init_irq		= miata_init_irq,
	.init_rtc		= common_init_rtc,
	.init_pci		= miata_init_pci,
	.kill_arch		= miata_kill_arch,
	.pci_map_irq		= miata_map_irq,
	.pci_swizzle		= miata_swizzle,
};
ALIAS_MV(miata)
