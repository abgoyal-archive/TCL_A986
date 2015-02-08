

#include "iwl-dev.h"
#include "iwl-core.h"
#include "iwl-debug.h"

#ifdef CONFIG_IWLWIFI_DEBUGFS
ssize_t iwl3945_ucode_rx_stats_read(struct file *file, char __user *user_buf,
				    size_t count, loff_t *ppos);
ssize_t iwl3945_ucode_tx_stats_read(struct file *file, char __user *user_buf,
				    size_t count, loff_t *ppos);
ssize_t iwl3945_ucode_general_stats_read(struct file *file,
					 char __user *user_buf, size_t count,
					 loff_t *ppos);
#else
static ssize_t iwl3945_ucode_rx_stats_read(struct file *file,
					   char __user *user_buf, size_t count,
					   loff_t *ppos)
{
	return 0;
}
static ssize_t iwl3945_ucode_tx_stats_read(struct file *file,
					   char __user *user_buf, size_t count,
					   loff_t *ppos)
{
	return 0;
}
static ssize_t iwl3945_ucode_general_stats_read(struct file *file,
						char __user *user_buf,
						size_t count, loff_t *ppos)
{
	return 0;
}
#endif
