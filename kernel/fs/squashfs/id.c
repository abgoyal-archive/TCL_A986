


#include <linux/fs.h>
#include <linux/vfs.h>
#include <linux/slab.h>

#include "squashfs_fs.h"
#include "squashfs_fs_sb.h"
#include "squashfs_fs_i.h"
#include "squashfs.h"

int squashfs_get_id(struct super_block *sb, unsigned int index,
					unsigned int *id)
{
	struct squashfs_sb_info *msblk = sb->s_fs_info;
	int block = SQUASHFS_ID_BLOCK(index);
	int offset = SQUASHFS_ID_BLOCK_OFFSET(index);
	u64 start_block = le64_to_cpu(msblk->id_table[block]);
	__le32 disk_id;
	int err;

	err = squashfs_read_metadata(sb, &disk_id, &start_block, &offset,
							sizeof(disk_id));
	if (err < 0)
		return err;

	*id = le32_to_cpu(disk_id);
	return 0;
}


__le64 *squashfs_read_id_index_table(struct super_block *sb,
			u64 id_table_start, unsigned short no_ids)
{
	unsigned int length = SQUASHFS_ID_BLOCK_BYTES(no_ids);
	__le64 *id_table;
	int err;

	TRACE("In read_id_index_table, length %d\n", length);

	/* Allocate id lookup table indexes */
	id_table = kmalloc(length, GFP_KERNEL);
	if (id_table == NULL) {
		ERROR("Failed to allocate id index table\n");
		return ERR_PTR(-ENOMEM);
	}

	err = squashfs_read_table(sb, id_table, id_table_start, length);
	if (err < 0) {
		ERROR("unable to read id index table\n");
		kfree(id_table);
		return ERR_PTR(err);
	}

	return id_table;
}
