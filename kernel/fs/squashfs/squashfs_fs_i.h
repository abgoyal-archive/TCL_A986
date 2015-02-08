
#ifndef SQUASHFS_FS_I
#define SQUASHFS_FS_I

struct squashfs_inode_info {
	u64		start;
	int		offset;
	u64		xattr;
	unsigned int	xattr_size;
	int		xattr_count;
	union {
		struct {
			u64		fragment_block;
			int		fragment_size;
			int		fragment_offset;
			u64		block_list_start;
		};
		struct {
			u64		dir_idx_start;
			int		dir_idx_offset;
			int		dir_idx_cnt;
			int		parent;
		};
	};
	struct inode	vfs_inode;
};
#endif
