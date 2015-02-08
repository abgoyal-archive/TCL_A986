
#include <linux/init.h>
#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/spinlock.h>
#include <asm/processor.h>
#include <asm/mmu_context.h>

static int asids_seq_show(struct seq_file *file, void *iter)
{
	struct task_struct *p;

	read_lock(&tasklist_lock);

	for_each_process(p) {
		int pid = p->pid;

		if (unlikely(!pid))
			continue;

		if (p->mm)
			seq_printf(file, "%5d : %04lx\n", pid,
				   cpu_asid(smp_processor_id(), p->mm));
	}

	read_unlock(&tasklist_lock);

	return 0;
}

static int asids_debugfs_open(struct inode *inode, struct file *file)
{
	return single_open(file, asids_seq_show, inode->i_private);
}

static const struct file_operations asids_debugfs_fops = {
	.owner		= THIS_MODULE,
	.open		= asids_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init asids_debugfs_init(void)
{
	struct dentry *asids_dentry;

	asids_dentry = debugfs_create_file("asids", S_IRUSR, sh_debugfs_root,
					   NULL, &asids_debugfs_fops);
	if (!asids_dentry)
		return -ENOMEM;
	if (IS_ERR(asids_dentry))
		return PTR_ERR(asids_dentry);

	return 0;
}
module_init(asids_debugfs_init);

MODULE_LICENSE("GPL v2");
