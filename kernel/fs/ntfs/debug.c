

#include "debug.h"

static char err_buf[1024];
static DEFINE_SPINLOCK(err_buf_lock);

void __ntfs_warning(const char *function, const struct super_block *sb,
		const char *fmt, ...)
{
	va_list args;
	int flen = 0;

#ifndef DEBUG
	if (!printk_ratelimit())
		return;
#endif
	if (function)
		flen = strlen(function);
	spin_lock(&err_buf_lock);
	va_start(args, fmt);
	vsnprintf(err_buf, sizeof(err_buf), fmt, args);
	va_end(args);
	if (sb)
		printk(KERN_ERR "NTFS-fs warning (device %s): %s(): %s\n",
				sb->s_id, flen ? function : "", err_buf);
	else
		printk(KERN_ERR "NTFS-fs warning: %s(): %s\n",
				flen ? function : "", err_buf);
	spin_unlock(&err_buf_lock);
}

void __ntfs_error(const char *function, const struct super_block *sb,
		const char *fmt, ...)
{
	va_list args;
	int flen = 0;

#ifndef DEBUG
	if (!printk_ratelimit())
		return;
#endif
	if (function)
		flen = strlen(function);
	spin_lock(&err_buf_lock);
	va_start(args, fmt);
	vsnprintf(err_buf, sizeof(err_buf), fmt, args);
	va_end(args);
	if (sb)
		printk(KERN_ERR "NTFS-fs error (device %s): %s(): %s\n",
				sb->s_id, flen ? function : "", err_buf);
	else
		printk(KERN_ERR "NTFS-fs error: %s(): %s\n",
				flen ? function : "", err_buf);
	spin_unlock(&err_buf_lock);
}

#ifdef DEBUG

/* If 1, output debug messages, and if 0, don't. */
int debug_msgs = 0;

void __ntfs_debug (const char *file, int line, const char *function,
		const char *fmt, ...)
{
	va_list args;
	int flen = 0;

	if (!debug_msgs)
		return;
	if (function)
		flen = strlen(function);
	spin_lock(&err_buf_lock);
	va_start(args, fmt);
	vsnprintf(err_buf, sizeof(err_buf), fmt, args);
	va_end(args);
	printk(KERN_DEBUG "NTFS-fs DEBUG (%s, %d): %s(): %s\n", file, line,
			flen ? function : "", err_buf);
	spin_unlock(&err_buf_lock);
}

/* Dump a runlist. Caller has to provide synchronisation for @rl. */
void ntfs_debug_dump_runlist(const runlist_element *rl)
{
	int i;
	const char *lcn_str[5] = { "LCN_HOLE         ", "LCN_RL_NOT_MAPPED",
				   "LCN_ENOENT       ", "LCN_unknown      " };

	if (!debug_msgs)
		return;
	printk(KERN_DEBUG "NTFS-fs DEBUG: Dumping runlist (values in hex):\n");
	if (!rl) {
		printk(KERN_DEBUG "Run list not present.\n");
		return;
	}
	printk(KERN_DEBUG "VCN              LCN               Run length\n");
	for (i = 0; ; i++) {
		LCN lcn = (rl + i)->lcn;

		if (lcn < (LCN)0) {
			int index = -lcn - 1;

			if (index > -LCN_ENOENT - 1)
				index = 3;
			printk(KERN_DEBUG "%-16Lx %s %-16Lx%s\n",
					(long long)(rl + i)->vcn, lcn_str[index],
					(long long)(rl + i)->length,
					(rl + i)->length ? "" :
						" (runlist end)");
		} else
			printk(KERN_DEBUG "%-16Lx %-16Lx  %-16Lx%s\n",
					(long long)(rl + i)->vcn,
					(long long)(rl + i)->lcn,
					(long long)(rl + i)->length,
					(rl + i)->length ? "" :
						" (runlist end)");
		if (!(rl + i)->length)
			break;
	}
}

#endif
