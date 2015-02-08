

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/nubus.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/init.h>
#include <linux/module.h>

#include <asm/uaccess.h>
#include <asm/byteorder.h>

static int
nubus_devices_proc_show(struct seq_file *m, void *v)
{
	struct nubus_dev *dev = nubus_devices;

	while (dev) {
		seq_printf(m, "%x\t%04x %04x %04x %04x",
			      dev->board->slot,
			      dev->category,
			      dev->type,
			      dev->dr_sw,
			      dev->dr_hw);
		seq_printf(m, "\t%08lx\n", dev->board->slot_addr);
		dev = dev->next;
	}
	return 0;
}

static int nubus_devices_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, nubus_devices_proc_show, NULL);
}

static const struct file_operations nubus_devices_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= nubus_devices_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static struct proc_dir_entry *proc_bus_nubus_dir;

static void nubus_proc_subdir(struct nubus_dev* dev,
			      struct proc_dir_entry* parent,
			      struct nubus_dir* dir)
{
	struct nubus_dirent ent;

	/* Some of these are directories, others aren't */
	while (nubus_readdir(dir, &ent) != -1) {
		char name[8];
		struct proc_dir_entry* e;
		
		sprintf(name, "%x", ent.type);
		e = create_proc_entry(name, S_IFREG | S_IRUGO |
				      S_IWUSR, parent);
		if (!e) return;
	}
}

static void nubus_proc_populate(struct nubus_dev* dev,
				struct proc_dir_entry* parent,
				struct nubus_dir* root)
{
	struct nubus_dirent ent;

	/* We know these are all directories (board resource + one or
	   more functional resources) */
	while (nubus_readdir(root, &ent) != -1) {
		char name[8];
		struct proc_dir_entry* e;
		struct nubus_dir dir;
		
		sprintf(name, "%x", ent.type);
		e = proc_mkdir(name, parent);
		if (!e) return;

		/* And descend */
		if (nubus_get_subdir(&ent, &dir) == -1) {
			/* This shouldn't happen */
			printk(KERN_ERR "NuBus root directory node %x:%x has no subdir!\n",
			       dev->board->slot, ent.type);
			continue;
		} else {
			nubus_proc_subdir(dev, e, &dir);
		}
	}
}

int nubus_proc_attach_device(struct nubus_dev *dev)
{
	struct proc_dir_entry *e;
	struct nubus_dir root;
	char name[8];

	if (dev == NULL) {
		printk(KERN_ERR
		       "NULL pointer in nubus_proc_attach_device, shoot the programmer!\n");
		return -1;
	}
		
	if (dev->board == NULL) {
		printk(KERN_ERR
		       "NULL pointer in nubus_proc_attach_device, shoot the programmer!\n");
		printk("dev = %p, dev->board = %p\n", dev, dev->board);
		return -1;
	}
		
	/* Create a directory */
	sprintf(name, "%x", dev->board->slot);
	e = dev->procdir = proc_mkdir(name, proc_bus_nubus_dir);
	if (!e)
		return -ENOMEM;

	/* Now recursively populate it with files */
	nubus_get_root_dir(dev->board, &root);
	nubus_proc_populate(dev, e, &root);

	return 0;
}
EXPORT_SYMBOL(nubus_proc_attach_device);

/* FIXME: this is certainly broken! */
int nubus_proc_detach_device(struct nubus_dev *dev)
{
	struct proc_dir_entry *e;

	if ((e = dev->procdir)) {
		if (atomic_read(&e->count))
			return -EBUSY;
		remove_proc_entry(e->name, proc_bus_nubus_dir);
		dev->procdir = NULL;
	}
	return 0;
}
EXPORT_SYMBOL(nubus_proc_detach_device);

void __init proc_bus_nubus_add_devices(void)
{
	struct nubus_dev *dev;
	
	for(dev = nubus_devices; dev; dev = dev->next)
		nubus_proc_attach_device(dev);
}

void __init nubus_proc_init(void)
{
	if (!MACH_IS_MAC)
		return;
	proc_bus_nubus_dir = proc_mkdir("bus/nubus", NULL);
	proc_create("devices", 0, proc_bus_nubus_dir, &nubus_devices_proc_fops);
	proc_bus_nubus_add_devices();
}
