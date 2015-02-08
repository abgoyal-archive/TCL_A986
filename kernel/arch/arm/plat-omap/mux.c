
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <asm/system.h>
#include <linux/spinlock.h>
#include <plat/mux.h>

#ifdef CONFIG_OMAP_MUX

static struct omap_mux_cfg *mux_cfg;

int __init omap_mux_register(struct omap_mux_cfg *arch_mux_cfg)
{
	if (!arch_mux_cfg || !arch_mux_cfg->pins || arch_mux_cfg->size == 0
			|| !arch_mux_cfg->cfg_reg) {
		printk(KERN_ERR "Invalid pin table\n");
		return -EINVAL;
	}

	mux_cfg = arch_mux_cfg;

	return 0;
}

int __init_or_module omap_cfg_reg(const unsigned long index)
{
	struct pin_config *reg;

	if (cpu_is_omap34xx() || cpu_is_omap44xx()) {
		printk(KERN_ERR "mux: Broken omap_cfg_reg(%lu) entry\n",
				index);
		WARN_ON(1);
		return -EINVAL;
	}

	if (mux_cfg == NULL) {
		printk(KERN_ERR "Pin mux table not initialized\n");
		return -ENODEV;
	}

	if (index >= mux_cfg->size) {
		printk(KERN_ERR "Invalid pin mux index: %lu (%lu)\n",
		       index, mux_cfg->size);
		dump_stack();
		return -ENODEV;
	}

	reg = (struct pin_config *)&mux_cfg->pins[index];

	if (!mux_cfg->cfg_reg)
		return -ENODEV;

	return mux_cfg->cfg_reg(reg);
}
EXPORT_SYMBOL(omap_cfg_reg);
#else
#define omap_mux_init() do {} while(0)
#define omap_cfg_reg(x)	do {} while(0)
#endif	/* CONFIG_OMAP_MUX */
