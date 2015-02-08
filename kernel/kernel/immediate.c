
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/immediate.h>
#include <linux/memory.h>
#include <linux/cpu.h>

#include <asm/sections.h>

static int imv_early_boot_complete;

extern struct __imv __start___imv[];
extern struct __imv __stop___imv[];

static DEFINE_MUTEX(imv_mutex);

void imv_update_range(const struct __imv *begin,
		const struct __imv *end)
{
	const struct __imv *iter;
	int ret;

	mutex_lock(&imv_mutex);
	for (iter = begin; iter < end; iter++) {
		if (!iter->imv) /* Skip removed __init immediate values */
			continue;
		/* workaround on_each_cpu cpu hotplug race */
		get_online_cpus();
		mutex_lock(&text_mutex);
		ret = arch_imv_update(iter, !imv_early_boot_complete);
		mutex_unlock(&text_mutex);
		put_online_cpus();
		if (imv_early_boot_complete && ret)
			printk(KERN_WARNING
				"Invalid immediate value. "
				"Variable at %p, "
				"instruction at %p, size %hu\n",
				(void *)iter->imv,
				(void *)iter->var, iter->size);
	}
	mutex_unlock(&imv_mutex);
}
EXPORT_SYMBOL_GPL(imv_update_range);

void core_imv_update(void)
{
	/* Core kernel imvs */
	imv_update_range(__start___imv, __stop___imv);
}
EXPORT_SYMBOL_GPL(core_imv_update);

void imv_unref(struct __imv *begin, struct __imv *end, void *start,
		unsigned long size)
{
	struct __imv *iter;

	for (iter = begin; iter < end; iter++)
		if (iter->imv >= (unsigned long)start
			&& iter->imv < (unsigned long)start + size)
			iter->imv = 0UL;
}

void imv_unref_core_init(void)
{
	imv_unref(__start___imv, __stop___imv, __init_begin,
		(unsigned long)__init_end - (unsigned long)__init_begin);
}

void __init imv_init_complete(void)
{
	imv_early_boot_complete = 1;
}

int imv_module_notify(struct notifier_block *self,
		      unsigned long val, void *data)
{
	struct module *mod = data;

	switch (val) {
	case MODULE_STATE_COMING:
		imv_update_range(mod->immediate,
				 mod->immediate + mod->num_immediate);
		break;
	case MODULE_STATE_GOING:
		/* All references will be gone, no update required. */
		break;
	}
	return 0;
}

struct notifier_block imv_module_nb = {
	.notifier_call = imv_module_notify,
	.priority = 0,
};

static int init_imv(void)
{
	return register_module_notifier(&imv_module_nb);
}
__initcall(init_imv);
