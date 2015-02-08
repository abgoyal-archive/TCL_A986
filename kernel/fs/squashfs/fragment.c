


#include <linux/fs.h>
#include <linux/vfs.h>
#include <linux/slab.h>

#include "squashfs_fs.h"
#include "squashfs_fs_sb.h"
#include "squashfs_fs_i.h"
#include "squashfs.h"

int squashfs_frag_lookup(struct super_block *sb, unsigned int fragment,
				u64 *fragment_block)
{
	struct squashfs_sb_info *msblk = sb->s_fs_info;
	int block = SQUASHFS_FRAGMENT_INDEX(fragment);
	int offset = SQUASHFS_FRAGMENT_INDEX_OFFSET(fragment);
	u64 start_block = le64_to_cpu(msblk->fragment_index[block]);
	struct squashfs_fragment_entry fragment_entry;
	int size;

	size = squashfs_read_metadata(sb, &fragment_entry, &start_block,
					&offset, sizeof(fragment_entry));
	if (size < 0)
		return size;

	*fragment_block = le64_to_cpu(fragment_entry.start_block);
	size = le32_to_cpu(fragment_entry.size);

	return size;
}


__le64 *squashfs_read_fragment_index_table(struct super_block *sb,
	u64 fragment_table_start, unsigned int fragments)
{
	unsigned int length = SQUASHFS_FRAGMENT_INDEX_BYTES(fragments);
	__le64 *fragment_index;
	int err;

	/* Allocate fragment lookup table indexes */
	fragment_index = kmalloc(length, GFP_KERNEL);
	if (fragment_index == NULL) {
		ERROR("Failed to allocate fragment index table\n");
		return ERR_PTR(-ENOMEM);
	}

	err = squashfs_read_table(sb, fragment_index, fragment_table_start,
			length);
	if (err < 0) {
		ERROR("unable to read fragment index table\n");
		kfree(fragment_index);
		return ERR_PTR(err);
	}

	return fragment_index;
}
