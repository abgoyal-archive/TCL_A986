

#include <linux/miscdevice.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/module.h>
#include <linux/init.h>
#include <net/net_namespace.h>

#include <net/irda/irda.h>
#include <net/irda/irlap.h>
#include <net/irda/irlmp.h>

extern const struct file_operations discovery_seq_fops;
extern const struct file_operations irlap_seq_fops;
extern const struct file_operations irlmp_seq_fops;
extern const struct file_operations irttp_seq_fops;
extern const struct file_operations irias_seq_fops;

struct irda_entry {
	const char *name;
	const struct file_operations *fops;
};

struct proc_dir_entry *proc_irda;
EXPORT_SYMBOL(proc_irda);

static const struct irda_entry irda_dirs[] = {
	{"discovery",	&discovery_seq_fops},
	{"irttp",	&irttp_seq_fops},
	{"irlmp",	&irlmp_seq_fops},
	{"irlap",	&irlap_seq_fops},
	{"irias",	&irias_seq_fops},
};

void __init irda_proc_register(void)
{
	int i;
	struct proc_dir_entry *d;

	proc_irda = proc_mkdir("irda", init_net.proc_net);
	if (proc_irda == NULL)
		return;

	for (i = 0; i < ARRAY_SIZE(irda_dirs); i++)
		d = proc_create(irda_dirs[i].name, 0, proc_irda,
				irda_dirs[i].fops);
}

void irda_proc_unregister(void)
{
	int i;

	if (proc_irda) {
		for (i=0; i<ARRAY_SIZE(irda_dirs); i++)
			remove_proc_entry(irda_dirs[i].name, proc_irda);

		remove_proc_entry("irda", init_net.proc_net);
		proc_irda = NULL;
	}
}


