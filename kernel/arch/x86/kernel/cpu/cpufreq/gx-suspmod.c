


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/smp.h>
#include <linux/cpufreq.h>
#include <linux/pci.h>
#include <linux/errno.h>
#include <linux/slab.h>

#include <asm/processor-cyrix.h>

/* PCI config registers, all at F0 */
#define PCI_PMER1	0x80	/* power management enable register 1 */
#define PCI_PMER2	0x81	/* power management enable register 2 */
#define PCI_PMER3	0x82	/* power management enable register 3 */
#define PCI_IRQTC	0x8c	/* irq speedup timer counter register:typical 2 to 4ms */
#define PCI_VIDTC	0x8d	/* video speedup timer counter register: typical 50 to 100ms */
#define PCI_MODOFF	0x94	/* suspend modulation OFF counter register, 1 = 32us */
#define PCI_MODON	0x95	/* suspend modulation ON counter register */
#define PCI_SUSCFG	0x96	/* suspend configuration register */

/* PMER1 bits */
#define GPM		(1<<0)	/* global power management */
#define GIT		(1<<1)	/* globally enable PM device idle timers */
#define GTR		(1<<2)	/* globally enable IO traps */
#define IRQ_SPDUP	(1<<3)	/* disable clock throttle during interrupt handling */
#define VID_SPDUP	(1<<4)	/* disable clock throttle during vga video handling */

/* SUSCFG bits */
#define SUSMOD		(1<<0)	/* enable/disable suspend modulation */
/* the below is supported only with cs5530 (after rev.1.2)/cs5530A */
#define SMISPDUP	(1<<1)	/* select how SMI re-enable suspend modulation: */
				/* IRQTC timer or read SMI speedup disable reg.(F1BAR[08-09h]) */
#define SUSCFG		(1<<2)	/* enable powering down a GXLV processor. "Special 3Volt Suspend" mode */
/* the below is supported only with cs5530A */
#define PWRSVE_ISA	(1<<3)	/* stop ISA clock  */
#define PWRSVE		(1<<4)	/* active idle */

struct gxfreq_params {
	u8 on_duration;
	u8 off_duration;
	u8 pci_suscfg;
	u8 pci_pmer1;
	u8 pci_pmer2;
	struct pci_dev *cs55x0;
};

static struct gxfreq_params *gx_params;
static int stock_freq;

/* PCI bus clock - defaults to 30.000 if cpu_khz is not available */
static int pci_busclk;
module_param(pci_busclk, int, 0444);

static int max_duration = 255;
module_param(max_duration, int, 0444);

#define POLICY_MIN_DIV 20


#define dprintk(msg...) cpufreq_debug_printk(CPUFREQ_DEBUG_DRIVER, \
		"gx-suspmod", msg)

static int gx_freq_mult[16] = {
		4, 10, 4, 6, 9, 5, 7, 8,
		0, 0, 0, 0, 0, 0, 0, 0
};


static struct pci_device_id gx_chipset_tbl[] __initdata = {
	{ PCI_VENDOR_ID_CYRIX, PCI_DEVICE_ID_CYRIX_5530_LEGACY,
		PCI_ANY_ID, PCI_ANY_ID },
	{ PCI_VENDOR_ID_CYRIX, PCI_DEVICE_ID_CYRIX_5520,
		PCI_ANY_ID, PCI_ANY_ID },
	{ PCI_VENDOR_ID_CYRIX, PCI_DEVICE_ID_CYRIX_5510,
		PCI_ANY_ID, PCI_ANY_ID },
	{ 0, },
};

static void gx_write_byte(int reg, int value)
{
	pci_write_config_byte(gx_params->cs55x0, reg, value);
}

static __init struct pci_dev *gx_detect_chipset(void)
{
	struct pci_dev *gx_pci = NULL;

	/* check if CPU is a MediaGX or a Geode. */
	if ((boot_cpu_data.x86_vendor != X86_VENDOR_NSC) &&
	    (boot_cpu_data.x86_vendor != X86_VENDOR_CYRIX)) {
		dprintk("error: no MediaGX/Geode processor found!\n");
		return NULL;
	}

	/* detect which companion chip is used */
	while ((gx_pci = pci_get_device(PCI_ANY_ID, PCI_ANY_ID, gx_pci)) != NULL) {
		if ((pci_match_id(gx_chipset_tbl, gx_pci)) != NULL)
			return gx_pci;
	}

	dprintk("error: no supported chipset found!\n");
	return NULL;
}

static unsigned int gx_get_cpuspeed(unsigned int cpu)
{
	if ((gx_params->pci_suscfg & SUSMOD) == 0)
		return stock_freq;

	return (stock_freq * gx_params->off_duration)
		/ (gx_params->on_duration + gx_params->off_duration);
}


static unsigned int gx_validate_speed(unsigned int khz, u8 *on_duration,
		u8 *off_duration)
{
	unsigned int i;
	u8 tmp_on, tmp_off;
	int old_tmp_freq = stock_freq;
	int tmp_freq;

	*off_duration = 1;
	*on_duration = 0;

	for (i = max_duration; i > 0; i--) {
		tmp_off = ((khz * i) / stock_freq) & 0xff;
		tmp_on = i - tmp_off;
		tmp_freq = (stock_freq * tmp_off) / i;
		/* if this relation is closer to khz, use this. If it's equal,
		 * prefer it, too - lower latency */
		if (abs(tmp_freq - khz) <= abs(old_tmp_freq - khz)) {
			*on_duration = tmp_on;
			*off_duration = tmp_off;
			old_tmp_freq = tmp_freq;
		}
	}

	return old_tmp_freq;
}



static void gx_set_cpuspeed(unsigned int khz)
{
	u8 suscfg, pmer1;
	unsigned int new_khz;
	unsigned long flags;
	struct cpufreq_freqs freqs;

	freqs.cpu = 0;
	freqs.old = gx_get_cpuspeed(0);

	new_khz = gx_validate_speed(khz, &gx_params->on_duration,
			&gx_params->off_duration);

	freqs.new = new_khz;

	cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);
	local_irq_save(flags);



	if (new_khz != stock_freq) {
		/* if new khz == 100% of CPU speed, it is special case */
		switch (gx_params->cs55x0->device) {
		case PCI_DEVICE_ID_CYRIX_5530_LEGACY:
			pmer1 = gx_params->pci_pmer1 | IRQ_SPDUP | VID_SPDUP;
			/* FIXME: need to test other values -- Zwane,Miura */
			/* typical 2 to 4ms */
			gx_write_byte(PCI_IRQTC, 4);
			/* typical 50 to 100ms */
			gx_write_byte(PCI_VIDTC, 100);
			gx_write_byte(PCI_PMER1, pmer1);

			if (gx_params->cs55x0->revision < 0x10) {
				/* CS5530(rev 1.2, 1.3) */
				suscfg = gx_params->pci_suscfg|SUSMOD;
			} else {
				/* CS5530A,B.. */
				suscfg = gx_params->pci_suscfg|SUSMOD|PWRSVE;
			}
			break;
		case PCI_DEVICE_ID_CYRIX_5520:
		case PCI_DEVICE_ID_CYRIX_5510:
			suscfg = gx_params->pci_suscfg | SUSMOD;
			break;
		default:
			local_irq_restore(flags);
			dprintk("fatal: try to set unknown chipset.\n");
			return;
		}
	} else {
		suscfg = gx_params->pci_suscfg & ~(SUSMOD);
		gx_params->off_duration = 0;
		gx_params->on_duration = 0;
		dprintk("suspend modulation disabled: cpu runs 100%% speed.\n");
	}

	gx_write_byte(PCI_MODOFF, gx_params->off_duration);
	gx_write_byte(PCI_MODON, gx_params->on_duration);

	gx_write_byte(PCI_SUSCFG, suscfg);
	pci_read_config_byte(gx_params->cs55x0, PCI_SUSCFG, &suscfg);

	local_irq_restore(flags);

	gx_params->pci_suscfg = suscfg;

	cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);

	dprintk("suspend modulation w/ duration of ON:%d us, OFF:%d us\n",
		gx_params->on_duration * 32, gx_params->off_duration * 32);
	dprintk("suspend modulation w/ clock speed: %d kHz.\n", freqs.new);
}



static int cpufreq_gx_verify(struct cpufreq_policy *policy)
{
	unsigned int tmp_freq = 0;
	u8 tmp1, tmp2;

	if (!stock_freq || !policy)
		return -EINVAL;

	policy->cpu = 0;
	cpufreq_verify_within_limits(policy, (stock_freq / max_duration),
			stock_freq);

	/* it needs to be assured that at least one supported frequency is
	 * within policy->min and policy->max. If it is not, policy->max
	 * needs to be increased until one freuqency is supported.
	 * policy->min may not be decreased, though. This way we guarantee a
	 * specific processing capacity.
	 */
	tmp_freq = gx_validate_speed(policy->min, &tmp1, &tmp2);
	if (tmp_freq < policy->min)
		tmp_freq += stock_freq / max_duration;
	policy->min = tmp_freq;
	if (policy->min > policy->max)
		policy->max = tmp_freq;
	tmp_freq = gx_validate_speed(policy->max, &tmp1, &tmp2);
	if (tmp_freq > policy->max)
		tmp_freq -= stock_freq / max_duration;
	policy->max = tmp_freq;
	if (policy->max < policy->min)
		policy->max = policy->min;
	cpufreq_verify_within_limits(policy, (stock_freq / max_duration),
			stock_freq);

	return 0;
}

static int cpufreq_gx_target(struct cpufreq_policy *policy,
			     unsigned int target_freq,
			     unsigned int relation)
{
	u8 tmp1, tmp2;
	unsigned int tmp_freq;

	if (!stock_freq || !policy)
		return -EINVAL;

	policy->cpu = 0;

	tmp_freq = gx_validate_speed(target_freq, &tmp1, &tmp2);
	while (tmp_freq < policy->min) {
		tmp_freq += stock_freq / max_duration;
		tmp_freq = gx_validate_speed(tmp_freq, &tmp1, &tmp2);
	}
	while (tmp_freq > policy->max) {
		tmp_freq -= stock_freq / max_duration;
		tmp_freq = gx_validate_speed(tmp_freq, &tmp1, &tmp2);
	}

	gx_set_cpuspeed(tmp_freq);

	return 0;
}

static int cpufreq_gx_cpu_init(struct cpufreq_policy *policy)
{
	unsigned int maxfreq, curfreq;

	if (!policy || policy->cpu != 0)
		return -ENODEV;

	/* determine maximum frequency */
	if (pci_busclk)
		maxfreq = pci_busclk * gx_freq_mult[getCx86(CX86_DIR1) & 0x0f];
	else if (cpu_khz)
		maxfreq = cpu_khz;
	else
		maxfreq = 30000 * gx_freq_mult[getCx86(CX86_DIR1) & 0x0f];

	stock_freq = maxfreq;
	curfreq = gx_get_cpuspeed(0);

	dprintk("cpu max frequency is %d.\n", maxfreq);
	dprintk("cpu current frequency is %dkHz.\n", curfreq);

	/* setup basic struct for cpufreq API */
	policy->cpu = 0;

	if (max_duration < POLICY_MIN_DIV)
		policy->min = maxfreq / max_duration;
	else
		policy->min = maxfreq / POLICY_MIN_DIV;
	policy->max = maxfreq;
	policy->cur = curfreq;
	policy->cpuinfo.min_freq = maxfreq / max_duration;
	policy->cpuinfo.max_freq = maxfreq;
	policy->cpuinfo.transition_latency = CPUFREQ_ETERNAL;

	return 0;
}

static struct cpufreq_driver gx_suspmod_driver = {
	.get		= gx_get_cpuspeed,
	.verify		= cpufreq_gx_verify,
	.target		= cpufreq_gx_target,
	.init		= cpufreq_gx_cpu_init,
	.name		= "gx-suspmod",
	.owner		= THIS_MODULE,
};

static int __init cpufreq_gx_init(void)
{
	int ret;
	struct gxfreq_params *params;
	struct pci_dev *gx_pci;

	/* Test if we have the right hardware */
	gx_pci = gx_detect_chipset();
	if (gx_pci == NULL)
		return -ENODEV;

	/* check whether module parameters are sane */
	if (max_duration > 0xff)
		max_duration = 0xff;

	dprintk("geode suspend modulation available.\n");

	params = kzalloc(sizeof(struct gxfreq_params), GFP_KERNEL);
	if (params == NULL)
		return -ENOMEM;

	params->cs55x0 = gx_pci;
	gx_params = params;

	/* keep cs55x0 configurations */
	pci_read_config_byte(params->cs55x0, PCI_SUSCFG, &(params->pci_suscfg));
	pci_read_config_byte(params->cs55x0, PCI_PMER1, &(params->pci_pmer1));
	pci_read_config_byte(params->cs55x0, PCI_PMER2, &(params->pci_pmer2));
	pci_read_config_byte(params->cs55x0, PCI_MODON, &(params->on_duration));
	pci_read_config_byte(params->cs55x0, PCI_MODOFF,
			&(params->off_duration));

	ret = cpufreq_register_driver(&gx_suspmod_driver);
	if (ret) {
		kfree(params);
		return ret;                   /* register error! */
	}

	return 0;
}

static void __exit cpufreq_gx_exit(void)
{
	cpufreq_unregister_driver(&gx_suspmod_driver);
	pci_dev_put(gx_params->cs55x0);
	kfree(gx_params);
}

MODULE_AUTHOR("Hiroshi Miura <miura@da-cha.org>");
MODULE_DESCRIPTION("Cpufreq driver for Cyrix MediaGX and NatSemi Geode");
MODULE_LICENSE("GPL");

module_init(cpufreq_gx_init);
module_exit(cpufreq_gx_exit);

