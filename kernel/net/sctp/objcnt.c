

#include <linux/kernel.h>
#include <net/sctp/sctp.h>


SCTP_DBG_OBJCNT(sock);
SCTP_DBG_OBJCNT(ep);
SCTP_DBG_OBJCNT(transport);
SCTP_DBG_OBJCNT(assoc);
SCTP_DBG_OBJCNT(bind_addr);
SCTP_DBG_OBJCNT(bind_bucket);
SCTP_DBG_OBJCNT(chunk);
SCTP_DBG_OBJCNT(addr);
SCTP_DBG_OBJCNT(ssnmap);
SCTP_DBG_OBJCNT(datamsg);
SCTP_DBG_OBJCNT(keys);

static sctp_dbg_objcnt_entry_t sctp_dbg_objcnt[] = {
	SCTP_DBG_OBJCNT_ENTRY(sock),
	SCTP_DBG_OBJCNT_ENTRY(ep),
	SCTP_DBG_OBJCNT_ENTRY(assoc),
	SCTP_DBG_OBJCNT_ENTRY(transport),
	SCTP_DBG_OBJCNT_ENTRY(chunk),
	SCTP_DBG_OBJCNT_ENTRY(bind_addr),
	SCTP_DBG_OBJCNT_ENTRY(bind_bucket),
	SCTP_DBG_OBJCNT_ENTRY(addr),
	SCTP_DBG_OBJCNT_ENTRY(ssnmap),
	SCTP_DBG_OBJCNT_ENTRY(datamsg),
	SCTP_DBG_OBJCNT_ENTRY(keys),
};

static int sctp_objcnt_seq_show(struct seq_file *seq, void *v)
{
	int i, len;

	i = (int)*(loff_t *)v;
	seq_printf(seq, "%s: %d%n", sctp_dbg_objcnt[i].label,
				atomic_read(sctp_dbg_objcnt[i].counter), &len);
	seq_printf(seq, "%*s\n", 127 - len, "");
	return 0;
}

static void *sctp_objcnt_seq_start(struct seq_file *seq, loff_t *pos)
{
	return (*pos >= ARRAY_SIZE(sctp_dbg_objcnt)) ? NULL : (void *)pos;
}

static void sctp_objcnt_seq_stop(struct seq_file *seq, void *v)
{
}

static void * sctp_objcnt_seq_next(struct seq_file *seq, void *v, loff_t *pos)
{
	++*pos;
	return (*pos >= ARRAY_SIZE(sctp_dbg_objcnt)) ? NULL : (void *)pos;
}

static const struct seq_operations sctp_objcnt_seq_ops = {
	.start = sctp_objcnt_seq_start,
	.next  = sctp_objcnt_seq_next,
	.stop  = sctp_objcnt_seq_stop,
	.show  = sctp_objcnt_seq_show,
};

static int sctp_objcnt_seq_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &sctp_objcnt_seq_ops);
}

static const struct file_operations sctp_objcnt_ops = {
	.open	 = sctp_objcnt_seq_open,
	.read	 = seq_read,
	.llseek	 = seq_lseek,
	.release = seq_release,
};

/* Initialize the objcount in the proc filesystem.  */
void sctp_dbg_objcnt_init(void)
{
	struct proc_dir_entry *ent;

	ent = proc_create("sctp_dbg_objcnt", 0,
			  proc_net_sctp, &sctp_objcnt_ops);
	if (!ent)
		printk(KERN_WARNING
			"sctp_dbg_objcnt: Unable to create /proc entry.\n");
}

/* Cleanup the objcount entry in the proc filesystem.  */
void sctp_dbg_objcnt_exit(void)
{
	remove_proc_entry("sctp_dbg_objcnt", proc_net_sctp);
}


