

#include <linux/fs.h>

#include "ufs_fs.h"
#include "ufs.h"

 
const struct file_operations ufs_file_operations = {
	.llseek		= generic_file_llseek,
	.read		= do_sync_read,
	.aio_read	= generic_file_aio_read,
	.write		= do_sync_write,
	.aio_write	= generic_file_aio_write,
	.mmap		= generic_file_mmap,
	.open           = generic_file_open,
	.fsync		= generic_file_fsync,
	.splice_read	= generic_file_splice_read,
};
