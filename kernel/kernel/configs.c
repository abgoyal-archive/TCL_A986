

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/init.h>
#include <asm/uaccess.h>

/**************************************************/
/* the actual current config file                 */

#define MAGIC_START	"IKCFG_ST"
#define MAGIC_END	"IKCFG_ED"
#include "config_data.h"


#define MAGIC_SIZE (sizeof(MAGIC_START) - 1)
#define kernel_config_data_size \
	(sizeof(kernel_config_data) - 1 - MAGIC_SIZE * 2)

#ifdef CONFIG_IKCONFIG_PROC

static ssize_t
ikconfig_read_current(struct file *file, char __user *buf,
		      size_t len, loff_t * offset)
{
	return simple_read_from_buffer(buf, len, offset,
				       kernel_config_data + MAGIC_SIZE,
				       kernel_config_data_size);
}

static const struct file_operations ikconfig_file_ops = {
	.owner = THIS_MODULE,
	.read = ikconfig_read_current,
};

static int __init ikconfig_init(void)
{
	struct proc_dir_entry *entry;

	/* create the current config file */
	entry = proc_create("config.gz", S_IFREG | S_IRUGO, NULL,
			    &ikconfig_file_ops);
	if (!entry)
		return -ENOMEM;

	entry->size = kernel_config_data_size;

	return 0;
}

static void __exit ikconfig_cleanup(void)
{
	remove_proc_entry("config.gz", NULL);
}

module_init(ikconfig_init);
module_exit(ikconfig_cleanup);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Randy Dunlap");
MODULE_DESCRIPTION("Echo the kernel .config file used to build the kernel");

#endif /* CONFIG_IKCONFIG_PROC */
