
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/bitops.h>
#include <linux/pci.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/io.h>

#include <asm/irq.h>
#include <asm/pgtable.h>
#include <asm/page.h>
#include <asm/system.h>
#include <mach/hardware.h>
#include <asm/mach-types.h>

#include <asm/mach/pci.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>
#include <asm/mach/time.h>
#include <asm/mach/flash.h>
#include <asm/mach/arch.h>

#include <mach/gpio.h>


static volatile unsigned long *board_irq_mask;
static volatile unsigned long *board_irq_stat;
static unsigned long board_irq_count;

#ifdef CONFIG_ARCH_IXDP2400
static struct slowport_cfg slowport_cpld_cfg = {
	.CCR =	SLOWPORT_CCR_DIV_2,
	.WTC = 0x00000070,
	.RTC = 0x00000070,
	.PCR = SLOWPORT_MODE_FLASH,
	.ADC = SLOWPORT_ADDR_WIDTH_24 | SLOWPORT_DATA_WIDTH_8
};
#endif

static void ixdp2x00_irq_mask(unsigned int irq)
{
	unsigned long dummy;
	static struct slowport_cfg old_cfg;

	/*
	 * This is ugly in common code but really don't know
	 * of a better way to handle it. :(
	 */
#ifdef CONFIG_ARCH_IXDP2400
	if (machine_is_ixdp2400())
		ixp2000_acquire_slowport(&slowport_cpld_cfg, &old_cfg);
#endif

	dummy = *board_irq_mask;
	dummy |=  IXP2000_BOARD_IRQ_MASK(irq);
	ixp2000_reg_wrb(board_irq_mask, dummy);

#ifdef CONFIG_ARCH_IXDP2400
	if (machine_is_ixdp2400())
		ixp2000_release_slowport(&old_cfg);
#endif
}

static void ixdp2x00_irq_unmask(unsigned int irq)
{
	unsigned long dummy;
	static struct slowport_cfg old_cfg;

#ifdef CONFIG_ARCH_IXDP2400
	if (machine_is_ixdp2400())
		ixp2000_acquire_slowport(&slowport_cpld_cfg, &old_cfg);
#endif

	dummy = *board_irq_mask;
	dummy &=  ~IXP2000_BOARD_IRQ_MASK(irq);
	ixp2000_reg_wrb(board_irq_mask, dummy);

	if (machine_is_ixdp2400()) 
		ixp2000_release_slowport(&old_cfg);
}

static void ixdp2x00_irq_handler(unsigned int irq, struct irq_desc *desc)
{
        volatile u32 ex_interrupt = 0;
	static struct slowport_cfg old_cfg;
	int i;

	desc->chip->mask(irq);

#ifdef CONFIG_ARCH_IXDP2400
	if (machine_is_ixdp2400())
		ixp2000_acquire_slowport(&slowport_cpld_cfg, &old_cfg);
#endif
        ex_interrupt = *board_irq_stat & 0xff;
	if (machine_is_ixdp2400())
		ixp2000_release_slowport(&old_cfg);

	if(!ex_interrupt) {
		printk(KERN_ERR "Spurious IXDP2x00 CPLD interrupt!\n");
		return;
	}

	for(i = 0; i < board_irq_count; i++) {
		if(ex_interrupt & (1 << i))  {
			int cpld_irq = IXP2000_BOARD_IRQ(0) + i;
			generic_handle_irq(cpld_irq);
		}
	}

	desc->chip->unmask(irq);
}

static struct irq_chip ixdp2x00_cpld_irq_chip = {
	.ack	= ixdp2x00_irq_mask,
	.mask	= ixdp2x00_irq_mask,
	.unmask	= ixdp2x00_irq_unmask
};

void __init ixdp2x00_init_irq(volatile unsigned long *stat_reg, volatile unsigned long *mask_reg, unsigned long nr_of_irqs)
{
	unsigned int irq;

	ixp2000_init_irq();
	
	if (!ixdp2x00_master_npu())
		return;

	board_irq_stat = stat_reg;
	board_irq_mask = mask_reg;
	board_irq_count = nr_of_irqs;

	*board_irq_mask = 0xffffffff;

	for(irq = IXP2000_BOARD_IRQ(0); irq < IXP2000_BOARD_IRQ(board_irq_count); irq++) {
		set_irq_chip(irq, &ixdp2x00_cpld_irq_chip);
		set_irq_handler(irq, handle_level_irq);
		set_irq_flags(irq, IRQF_VALID);
	}

	/* Hook into PCI interrupt */
	set_irq_chained_handler(IRQ_IXP2000_PCIB, ixdp2x00_irq_handler);
}

static struct map_desc ixdp2x00_io_desc __initdata = {
	.virtual	= IXDP2X00_VIRT_CPLD_BASE, 
	.pfn		= __phys_to_pfn(IXDP2X00_PHYS_CPLD_BASE),
	.length		= IXDP2X00_CPLD_SIZE,
	.type		= MT_DEVICE
};

void __init ixdp2x00_map_io(void)
{
	ixp2000_map_io();	

	iotable_init(&ixdp2x00_io_desc, 1);
}

void ixdp2x00_slave_pci_postinit(void)
{
	struct pci_dev *dev;

	/*
	 * Remove PMC device is there is one
	 */
	if((dev = pci_get_bus_and_slot(1, IXDP2X00_PMC_DEVFN))) {
		pci_remove_bus_device(dev);
		pci_dev_put(dev);
	}

	dev = pci_get_bus_and_slot(0, IXDP2X00_21555_DEVFN);
	pci_remove_bus_device(dev);
	pci_dev_put(dev);
}

static struct flash_platform_data ixdp2x00_platform_data = {
	.map_name	= "cfi_probe",
	.width		= 1,
};

static struct ixp2000_flash_data ixdp2x00_flash_data = {
	.platform_data	= &ixdp2x00_platform_data,
	.nr_banks	= 1
};

static struct resource ixdp2x00_flash_resource = {
	.start		= 0xc4000000,
	.end		= 0xc4000000 + 0x00ffffff,
	.flags		= IORESOURCE_MEM,
};

static struct platform_device ixdp2x00_flash = {
	.name		= "IXP2000-Flash",
	.id		= 0,
	.dev		= {
		.platform_data = &ixdp2x00_flash_data,
	},
	.num_resources	= 1,
	.resource	= &ixdp2x00_flash_resource,
};

static struct ixp2000_i2c_pins ixdp2x00_i2c_gpio_pins = {
	.sda_pin	= IXDP2X00_GPIO_SDA,
	.scl_pin	= IXDP2X00_GPIO_SCL,
};

static struct platform_device ixdp2x00_i2c_controller = {
	.name		= "IXP2000-I2C",
	.id		= 0,
	.dev		= {
		.platform_data = &ixdp2x00_i2c_gpio_pins,
	},
	.num_resources	= 0
};

static struct platform_device *ixdp2x00_devices[] __initdata = {
	&ixdp2x00_flash,
	&ixdp2x00_i2c_controller
};

void __init ixdp2x00_init_machine(void)
{
	gpio_line_set(IXDP2X00_GPIO_I2C_ENABLE, 1);
	gpio_line_config(IXDP2X00_GPIO_I2C_ENABLE, GPIO_OUT);

	platform_add_devices(ixdp2x00_devices, ARRAY_SIZE(ixdp2x00_devices));
	ixp2000_uart_init();
}

