

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/ide.h>
#include <linux/platform_device.h>

#include <mach/board.h>
#include <mach/gpio.h>
#include <mach/at91sam9_smc.h>

#define DRV_NAME "at91_ide"

#define perr(fmt, args...) pr_err(DRV_NAME ": " fmt, ##args)
#define pdbg(fmt, args...) pr_debug("%s " fmt, __func__, ##args)


#define TASK_FILE	0x00c00000
#define ALT_MODE	0x00e00000
#define REGS_SIZE	8

#define enter_16bit(cs, mode) do {					\
	mode = at91_sys_read(AT91_SMC_MODE(cs));			\
	at91_sys_write(AT91_SMC_MODE(cs), mode | AT91_SMC_DBW_16);	\
} while (0)

#define leave_16bit(cs, mode) at91_sys_write(AT91_SMC_MODE(cs), mode);

static void set_smc_timings(const u8 chipselect, const u16 cycle,
			    const u16 setup, const u16 pulse,
			    const u16 data_float, int use_iordy)
{
	unsigned long mode = AT91_SMC_READMODE | AT91_SMC_WRITEMODE |
			     AT91_SMC_BAT_SELECT;

	/* disable or enable waiting for IORDY signal */
	if (use_iordy)
		mode |= AT91_SMC_EXNWMODE_READY;

	/* add data float cycles if needed */
	if (data_float)
		mode |= AT91_SMC_TDF_(data_float);

	at91_sys_write(AT91_SMC_MODE(chipselect), mode);

	/* setup timings in SMC */
	at91_sys_write(AT91_SMC_SETUP(chipselect), AT91_SMC_NWESETUP_(setup) |
						   AT91_SMC_NCS_WRSETUP_(0) |
						   AT91_SMC_NRDSETUP_(setup) |
						   AT91_SMC_NCS_RDSETUP_(0));
	at91_sys_write(AT91_SMC_PULSE(chipselect), AT91_SMC_NWEPULSE_(pulse) |
						   AT91_SMC_NCS_WRPULSE_(cycle) |
						   AT91_SMC_NRDPULSE_(pulse) |
						   AT91_SMC_NCS_RDPULSE_(cycle));
	at91_sys_write(AT91_SMC_CYCLE(chipselect), AT91_SMC_NWECYCLE_(cycle) |
						   AT91_SMC_NRDCYCLE_(cycle));
}

static unsigned int calc_mck_cycles(unsigned int ns, unsigned int mck_hz)
{
	u64 tmp = ns;

	tmp *= mck_hz;
	tmp += 1000*1000*1000 - 1; /* round up */
	do_div(tmp, 1000*1000*1000);
	return (unsigned int) tmp;
}

static void apply_timings(const u8 chipselect, const u8 pio,
			  const struct ide_timing *timing, int use_iordy)
{
	unsigned int t0, t1, t2, t6z;
	unsigned int cycle, setup, pulse, data_float;
	unsigned int mck_hz;
	struct clk *mck;

	/* see table 22 of Compact Flash standard 4.1 for the meaning,
	 * we do not stretch active (t2) time, so setup (t1) + hold time (th)
	 * assure at least minimal recovery (t2i) time */
	t0 = timing->cyc8b;
	t1 = timing->setup;
	t2 = timing->act8b;
	t6z = (pio < 5) ? 30 : 20;

	pdbg("t0=%u t1=%u t2=%u t6z=%u\n", t0, t1, t2, t6z);

	mck = clk_get(NULL, "mck");
	BUG_ON(IS_ERR(mck));
	mck_hz = clk_get_rate(mck);
	pdbg("mck_hz=%u\n", mck_hz);

	cycle = calc_mck_cycles(t0, mck_hz);
	setup = calc_mck_cycles(t1, mck_hz);
	pulse = calc_mck_cycles(t2, mck_hz);
	data_float = calc_mck_cycles(t6z, mck_hz);

	pdbg("cycle=%u setup=%u pulse=%u data_float=%u\n",
	     cycle, setup, pulse, data_float);

	set_smc_timings(chipselect, cycle, setup, pulse, data_float, use_iordy);
}

static void at91_ide_input_data(ide_drive_t *drive, struct ide_cmd *cmd,
				void *buf, unsigned int len)
{
	ide_hwif_t *hwif = drive->hwif;
	struct ide_io_ports *io_ports = &hwif->io_ports;
	u8 chipselect = hwif->select_data;
	unsigned long mode;

	pdbg("cs %u buf %p len %d\n", chipselect, buf, len);

	len++;

	enter_16bit(chipselect, mode);
	readsw((void __iomem *)io_ports->data_addr, buf, len / 2);
	leave_16bit(chipselect, mode);
}

static void at91_ide_output_data(ide_drive_t *drive, struct ide_cmd *cmd,
				 void *buf, unsigned int len)
{
	ide_hwif_t *hwif = drive->hwif;
	struct ide_io_ports *io_ports = &hwif->io_ports;
	u8 chipselect = hwif->select_data;
	unsigned long mode;

	pdbg("cs %u buf %p len %d\n", chipselect,  buf, len);

	enter_16bit(chipselect, mode);
	writesw((void __iomem *)io_ports->data_addr, buf, len / 2);
	leave_16bit(chipselect, mode);
}

static void at91_ide_set_pio_mode(ide_hwif_t *hwif, ide_drive_t *drive)
{
	struct ide_timing *timing;
	u8 chipselect = hwif->select_data;
	int use_iordy = 0;
	const u8 pio = drive->pio_mode - XFER_PIO_0;

	pdbg("chipselect %u pio %u\n", chipselect, pio);

	timing = ide_timing_find_mode(XFER_PIO_0 + pio);
	BUG_ON(!timing);

	if (ide_pio_need_iordy(drive, pio))
		use_iordy = 1;

	apply_timings(chipselect, pio, timing, use_iordy);
}

static const struct ide_tp_ops at91_ide_tp_ops = {
	.exec_command	= ide_exec_command,
	.read_status	= ide_read_status,
	.read_altstatus	= ide_read_altstatus,
	.write_devctl	= ide_write_devctl,

	.dev_select	= ide_dev_select,
	.tf_load	= ide_tf_load,
	.tf_read	= ide_tf_read,

	.input_data	= at91_ide_input_data,
	.output_data	= at91_ide_output_data,
};

static const struct ide_port_ops at91_ide_port_ops = {
	.set_pio_mode	= at91_ide_set_pio_mode,
};

static const struct ide_port_info at91_ide_port_info __initdata = {
	.port_ops	= &at91_ide_port_ops,
	.tp_ops		= &at91_ide_tp_ops,
	.host_flags 	= IDE_HFLAG_MMIO | IDE_HFLAG_NO_DMA | IDE_HFLAG_SINGLE |
			  IDE_HFLAG_NO_IO_32BIT | IDE_HFLAG_UNMASK_IRQS,
	.pio_mask 	= ATA_PIO6,
	.chipset	= ide_generic,
};


irqreturn_t at91_irq_handler(int irq, void *dev_id)
{
	int ntries = 8;
	int pin_val1, pin_val2;

	/* additional deglitch, line can be noisy in badly designed PCB */
	do {
		pin_val1 = at91_get_gpio_value(irq);
		pin_val2 = at91_get_gpio_value(irq);
	} while (pin_val1 != pin_val2 && --ntries > 0);

	if (pin_val1 == 0 || ntries <= 0)
		return IRQ_HANDLED;

	return ide_intr(irq, dev_id);
}

static int __init at91_ide_probe(struct platform_device *pdev)
{
	int ret;
	struct ide_hw hw, *hws[] = { &hw };
	struct ide_host *host;
	struct resource *res;
	unsigned long tf_base = 0, ctl_base = 0;
	struct at91_cf_data *board = pdev->dev.platform_data;

	if (!board)
		return -ENODEV;

	if (board->det_pin && at91_get_gpio_value(board->det_pin) != 0) {
		perr("no device detected\n");
		return -ENODEV;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		perr("can't get memory resource\n");
		return -ENODEV;
	}

	if (!devm_request_mem_region(&pdev->dev, res->start + TASK_FILE,
				     REGS_SIZE, "ide") ||
	    !devm_request_mem_region(&pdev->dev, res->start + ALT_MODE,
				     REGS_SIZE, "alt")) {
		perr("memory resources in use\n");
		return -EBUSY;
	}

	pdbg("chipselect %u irq %u res %08lx\n", board->chipselect,
	     board->irq_pin, (unsigned long) res->start);

	tf_base = (unsigned long) devm_ioremap(&pdev->dev, res->start + TASK_FILE,
					       REGS_SIZE);
	ctl_base = (unsigned long) devm_ioremap(&pdev->dev, res->start + ALT_MODE,
						REGS_SIZE);
	if (!tf_base || !ctl_base) {
		perr("can't map memory regions\n");
		return -EBUSY;
	}

	memset(&hw, 0, sizeof(hw));

	if (board->flags & AT91_IDE_SWAP_A0_A2) {
		/* workaround for stupid hardware bug */
		hw.io_ports.data_addr	= tf_base + 0;
		hw.io_ports.error_addr	= tf_base + 4;
		hw.io_ports.nsect_addr	= tf_base + 2;
		hw.io_ports.lbal_addr	= tf_base + 6;
		hw.io_ports.lbam_addr	= tf_base + 1;
		hw.io_ports.lbah_addr	= tf_base + 5;
		hw.io_ports.device_addr = tf_base + 3;
		hw.io_ports.command_addr = tf_base + 7;
		hw.io_ports.ctl_addr	= ctl_base + 3;
	} else
		ide_std_init_ports(&hw, tf_base, ctl_base + 6);

	hw.irq = board->irq_pin;
	hw.dev = &pdev->dev;

	host = ide_host_alloc(&at91_ide_port_info, hws, 1);
	if (!host) {
		perr("failed to allocate ide host\n");
		return -ENOMEM;
	}

	/* setup Static Memory Controller - PIO 0 as default */
	apply_timings(board->chipselect, 0, ide_timing_find_mode(XFER_PIO_0), 0);

	/* with GPIO interrupt we have to do quirks in handler */
	if (board->irq_pin >= PIN_BASE)
		host->irq_handler = at91_irq_handler;

	host->ports[0]->select_data = board->chipselect;

	ret = ide_host_register(host, &at91_ide_port_info, hws);
	if (ret) {
		perr("failed to register ide host\n");
		goto err_free_host;
	}
	platform_set_drvdata(pdev, host);
	return 0;

err_free_host:
	ide_host_free(host);
	return ret;
}

static int __exit at91_ide_remove(struct platform_device *pdev)
{
	struct ide_host *host = platform_get_drvdata(pdev);

	ide_host_remove(host);
	return 0;
}

static struct platform_driver at91_ide_driver = {
	.driver	= {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
	},
	.remove	= __exit_p(at91_ide_remove),
};

static int __init at91_ide_init(void)
{
	return platform_driver_probe(&at91_ide_driver, at91_ide_probe);
}

static void __exit at91_ide_exit(void)
{
	platform_driver_unregister(&at91_ide_driver);
}

module_init(at91_ide_init);
module_exit(at91_ide_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Stanislaw Gruszka <stf_xl@wp.pl>");

