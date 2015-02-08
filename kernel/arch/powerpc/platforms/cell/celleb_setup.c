

#undef DEBUG

#include <linux/cpu.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/stddef.h>
#include <linux/unistd.h>
#include <linux/reboot.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/seq_file.h>
#include <linux/root_dev.h>
#include <linux/console.h>
#include <linux/of_platform.h>

#include <asm/mmu.h>
#include <asm/processor.h>
#include <asm/io.h>
#include <asm/prom.h>
#include <asm/machdep.h>
#include <asm/cputable.h>
#include <asm/irq.h>
#include <asm/time.h>
#include <asm/spu_priv1.h>
#include <asm/firmware.h>
#include <asm/rtas.h>
#include <asm/cell-regs.h>

#include "beat_interrupt.h"
#include "beat_wrapper.h"
#include "beat.h"
#include "celleb_pci.h"
#include "interrupt.h"
#include "pervasive.h"
#include "ras.h"

static char celleb_machine_type[128] = "Celleb";

static void celleb_show_cpuinfo(struct seq_file *m)
{
	struct device_node *root;
	const char *model = "";

	root = of_find_node_by_path("/");
	if (root)
		model = of_get_property(root, "model", NULL);
	/* using "CHRP" is to trick anaconda into installing FCx into Celleb */
	seq_printf(m, "machine\t\t: %s %s\n", celleb_machine_type, model);
	of_node_put(root);
}

static int __init celleb_machine_type_hack(char *ptr)
{
	strlcpy(celleb_machine_type, ptr, sizeof(celleb_machine_type));
	return 0;
}

__setup("celleb_machine_type_hack=", celleb_machine_type_hack);

static void celleb_progress(char *s, unsigned short hex)
{
	printk("*** %04x : %s\n", hex, s ? s : "");
}

static void __init celleb_setup_arch_common(void)
{
	/* init to some ~sane value until calibrate_delay() runs */
	loops_per_jiffy = 50000000;

#ifdef CONFIG_DUMMY_CONSOLE
	conswitchp = &dummy_con;
#endif
}

static struct of_device_id celleb_bus_ids[] __initdata = {
	{ .type = "scc", },
	{ .type = "ioif", },	/* old style */
	{},
};

static int __init celleb_publish_devices(void)
{
	/* Publish OF platform devices for southbridge IOs */
	of_platform_bus_probe(NULL, celleb_bus_ids, NULL);

	return 0;
}
machine_device_initcall(celleb_beat, celleb_publish_devices);
machine_device_initcall(celleb_native, celleb_publish_devices);


static void __init celleb_setup_arch_beat(void)
{
#ifdef CONFIG_SPU_BASE
	spu_priv1_ops		= &spu_priv1_beat_ops;
	spu_management_ops	= &spu_management_of_ops;
#endif

#ifdef CONFIG_SMP
	smp_init_celleb();
#endif

	celleb_setup_arch_common();
}

static int __init celleb_probe_beat(void)
{
	unsigned long root = of_get_flat_dt_root();

	if (!of_flat_dt_is_compatible(root, "Beat"))
		return 0;

	powerpc_firmware_features |= FW_FEATURE_CELLEB_ALWAYS
		| FW_FEATURE_BEAT | FW_FEATURE_LPAR;
	hpte_init_beat_v3();

	return 1;
}


static void __init celleb_init_IRQ_native(void)
{
	iic_init_IRQ();
	spider_init_IRQ();
}

static void __init celleb_setup_arch_native(void)
{
#ifdef CONFIG_SPU_BASE
	spu_priv1_ops		= &spu_priv1_mmio_ops;
	spu_management_ops	= &spu_management_of_ops;
#endif

	cbe_regs_init();

#ifdef CONFIG_CBE_RAS
	cbe_ras_init();
#endif

#ifdef CONFIG_SMP
	smp_init_cell();
#endif

	cbe_pervasive_init();

	/* XXX: nvram initialization should be added */

	celleb_setup_arch_common();
}

static int __init celleb_probe_native(void)
{
	unsigned long root = of_get_flat_dt_root();

	if (of_flat_dt_is_compatible(root, "Beat") ||
	    !of_flat_dt_is_compatible(root, "TOSHIBA,Celleb"))
		return 0;

	powerpc_firmware_features |= FW_FEATURE_CELLEB_ALWAYS;
	hpte_init_native();

	return 1;
}


define_machine(celleb_beat) {
	.name			= "Cell Reference Set (Beat)",
	.probe			= celleb_probe_beat,
	.setup_arch		= celleb_setup_arch_beat,
	.show_cpuinfo		= celleb_show_cpuinfo,
	.restart		= beat_restart,
	.power_off		= beat_power_off,
	.halt			= beat_halt,
	.get_rtc_time		= beat_get_rtc_time,
	.set_rtc_time		= beat_set_rtc_time,
	.calibrate_decr		= generic_calibrate_decr,
	.progress		= celleb_progress,
	.power_save		= beat_power_save,
	.nvram_size		= beat_nvram_get_size,
	.nvram_read		= beat_nvram_read,
	.nvram_write		= beat_nvram_write,
	.set_dabr		= beat_set_xdabr,
	.init_IRQ		= beatic_init_IRQ,
	.get_irq		= beatic_get_irq,
	.pci_probe_mode 	= celleb_pci_probe_mode,
	.pci_setup_phb		= celleb_setup_phb,
#ifdef CONFIG_KEXEC
	.kexec_cpu_down		= beat_kexec_cpu_down,
#endif
};

define_machine(celleb_native) {
	.name			= "Cell Reference Set (native)",
	.probe			= celleb_probe_native,
	.setup_arch		= celleb_setup_arch_native,
	.show_cpuinfo		= celleb_show_cpuinfo,
	.restart		= rtas_restart,
	.power_off		= rtas_power_off,
	.halt			= rtas_halt,
	.get_boot_time		= rtas_get_boot_time,
	.get_rtc_time		= rtas_get_rtc_time,
	.set_rtc_time		= rtas_set_rtc_time,
	.calibrate_decr		= generic_calibrate_decr,
	.progress		= celleb_progress,
	.pci_probe_mode 	= celleb_pci_probe_mode,
	.pci_setup_phb		= celleb_setup_phb,
	.init_IRQ		= celleb_init_IRQ_native,
};
