

#ifndef _LINUX_NTFS_BITMAP_H
#define _LINUX_NTFS_BITMAP_H

#ifdef NTFS_RW

#include <linux/fs.h>

#include "types.h"

extern int __ntfs_bitmap_set_bits_in_run(struct inode *vi, const s64 start_bit,
		const s64 count, const u8 value, const bool is_rollback);

static inline int ntfs_bitmap_set_bits_in_run(struct inode *vi,
		const s64 start_bit, const s64 count, const u8 value)
{
	return __ntfs_bitmap_set_bits_in_run(vi, start_bit, count, value,
			false);
}

static inline int ntfs_bitmap_set_run(struct inode *vi, const s64 start_bit,
		const s64 count)
{
	return ntfs_bitmap_set_bits_in_run(vi, start_bit, count, 1);
}

static inline int ntfs_bitmap_clear_run(struct inode *vi, const s64 start_bit,
		const s64 count)
{
	return ntfs_bitmap_set_bits_in_run(vi, start_bit, count, 0);
}

static inline int ntfs_bitmap_set_bit(struct inode *vi, const s64 bit)
{
	return ntfs_bitmap_set_run(vi, bit, 1);
}

static inline int ntfs_bitmap_clear_bit(struct inode *vi, const s64 bit)
{
	return ntfs_bitmap_clear_run(vi, bit, 1);
}

#endif /* NTFS_RW */

#endif /* defined _LINUX_NTFS_BITMAP_H */
