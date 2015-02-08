
#include <linux/pci.h>
#include <linux/msi.h>
#include <asm/mach/irq.h>
#include <asm/irq.h>


#define IOP13XX_NUM_MSI_IRQS 128
static DECLARE_BITMAP(msi_irq_in_use, IOP13XX_NUM_MSI_IRQS);

static u32 read_imipr_0(void)
{
	u32 val;
	asm volatile("mrc p6, 0, %0, c8, c1, 0":"=r" (val));
	return val;
}
static void write_imipr_0(u32 val)
{
	asm volatile("mcr p6, 0, %0, c8, c1, 0"::"r" (val));
}

static u32 read_imipr_1(void)
{
	u32 val;
	asm volatile("mrc p6, 0, %0, c9, c1, 0":"=r" (val));
	return val;
}
static void write_imipr_1(u32 val)
{
	asm volatile("mcr p6, 0, %0, c9, c1, 0"::"r" (val));
}

static u32 read_imipr_2(void)
{
	u32 val;
	asm volatile("mrc p6, 0, %0, c10, c1, 0":"=r" (val));
	return val;
}
static void write_imipr_2(u32 val)
{
	asm volatile("mcr p6, 0, %0, c10, c1, 0"::"r" (val));
}

static u32 read_imipr_3(void)
{
	u32 val;
	asm volatile("mrc p6, 0, %0, c11, c1, 0":"=r" (val));
	return val;
}
static void write_imipr_3(u32 val)
{
	asm volatile("mcr p6, 0, %0, c11, c1, 0"::"r" (val));
}

static u32 (*read_imipr[])(void) = {
	read_imipr_0,
	read_imipr_1,
	read_imipr_2,
	read_imipr_3,
};

static void (*write_imipr[])(u32) = {
	write_imipr_0,
	write_imipr_1,
	write_imipr_2,
	write_imipr_3,
};

static void iop13xx_msi_handler(unsigned int irq, struct irq_desc *desc)
{
	int i, j;
	unsigned long status;

	/* read IMIPR registers and find any active interrupts,
	 * then call ISR for each active interrupt
	 */
	for (i = 0; i < ARRAY_SIZE(read_imipr); i++) {
		status = (read_imipr[i])();
		if (!status)
			continue;

		do {
			j = find_first_bit(&status, 32);
			(write_imipr[i])(1 << j); /* write back to clear bit */
			generic_handle_irq(IRQ_IOP13XX_MSI_0 + j + (32*i));
			status = (read_imipr[i])();
		} while (status);
	}
}

void __init iop13xx_msi_init(void)
{
	set_irq_chained_handler(IRQ_IOP13XX_INBD_MSI, iop13xx_msi_handler);
}

int create_irq(void)
{
	int irq, pos;

again:
	pos = find_first_zero_bit(msi_irq_in_use, IOP13XX_NUM_MSI_IRQS);
	irq = IRQ_IOP13XX_MSI_0 + pos;
	if (irq > NR_IRQS)
		return -ENOSPC;
	/* test_and_set_bit operates on 32-bits at a time */
	if (test_and_set_bit(pos, msi_irq_in_use))
		goto again;

	dynamic_irq_init(irq);

	return irq;
}

void destroy_irq(unsigned int irq)
{
	int pos = irq - IRQ_IOP13XX_MSI_0;

	dynamic_irq_cleanup(irq);

	clear_bit(pos, msi_irq_in_use);
}

void arch_teardown_msi_irq(unsigned int irq)
{
	destroy_irq(irq);
}

static void iop13xx_msi_nop(unsigned int irq)
{
	return;
}

static struct irq_chip iop13xx_msi_chip = {
	.name = "PCI-MSI",
	.ack = iop13xx_msi_nop,
	.enable = unmask_msi_irq,
	.disable = mask_msi_irq,
	.mask = mask_msi_irq,
	.unmask = unmask_msi_irq,
};

int arch_setup_msi_irq(struct pci_dev *pdev, struct msi_desc *desc)
{
	int id, irq = create_irq();
	struct msi_msg msg;

	if (irq < 0)
		return irq;

	set_irq_msi(irq, desc);

	msg.address_hi = 0x0;
	msg.address_lo = IOP13XX_MU_MIMR_PCI;

	id = iop13xx_cpu_id();
	msg.data = (id << IOP13XX_MU_MIMR_CORE_SELECT) | (irq & 0x7f);

	write_msi_msg(irq, &msg);
	set_irq_chip_and_handler(irq, &iop13xx_msi_chip, handle_simple_irq);

	return 0;
}
