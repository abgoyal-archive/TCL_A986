
#ifndef LINUX_EXPORTFS_H
#define LINUX_EXPORTFS_H 1

#include <linux/types.h>

struct dentry;
struct inode;
struct super_block;
struct vfsmount;

enum fid_type {
	/*
	 * The root, or export point, of the filesystem.
	 * (Never actually passed down to the filesystem.
	 */
	FILEID_ROOT = 0,

	/*
	 * 32bit inode number, 32 bit generation number.
	 */
	FILEID_INO32_GEN = 1,

	/*
	 * 32bit inode number, 32 bit generation number,
	 * 32 bit parent directory inode number.
	 */
	FILEID_INO32_GEN_PARENT = 2,

	/*
	 * 64 bit object ID, 64 bit root object ID,
	 * 32 bit generation number.
	 */
	FILEID_BTRFS_WITHOUT_PARENT = 0x4d,

	/*
	 * 64 bit object ID, 64 bit root object ID,
	 * 32 bit generation number,
	 * 64 bit parent object ID, 32 bit parent generation.
	 */
	FILEID_BTRFS_WITH_PARENT = 0x4e,

	/*
	 * 64 bit object ID, 64 bit root object ID,
	 * 32 bit generation number,
	 * 64 bit parent object ID, 32 bit parent generation,
	 * 64 bit parent root object ID.
	 */
	FILEID_BTRFS_WITH_PARENT_ROOT = 0x4f,

	/*
	 * 32 bit block number, 16 bit partition reference,
	 * 16 bit unused, 32 bit generation number.
	 */
	FILEID_UDF_WITHOUT_PARENT = 0x51,

	/*
	 * 32 bit block number, 16 bit partition reference,
	 * 16 bit unused, 32 bit generation number,
	 * 32 bit parent block number, 32 bit parent generation number
	 */
	FILEID_UDF_WITH_PARENT = 0x52,
};

struct fid {
	union {
		struct {
			u32 ino;
			u32 gen;
			u32 parent_ino;
			u32 parent_gen;
		} i32;
 		struct {
 			u32 block;
 			u16 partref;
 			u16 parent_partref;
 			u32 generation;
 			u32 parent_block;
 			u32 parent_generation;
 		} udf;
		__u32 raw[0];
	};
};


struct export_operations {
	int (*encode_fh)(struct dentry *de, __u32 *fh, int *max_len,
			int connectable);
	struct dentry * (*fh_to_dentry)(struct super_block *sb, struct fid *fid,
			int fh_len, int fh_type);
	struct dentry * (*fh_to_parent)(struct super_block *sb, struct fid *fid,
			int fh_len, int fh_type);
	int (*get_name)(struct dentry *parent, char *name,
			struct dentry *child);
	struct dentry * (*get_parent)(struct dentry *child);
	int (*commit_metadata)(struct inode *inode);
};

extern int exportfs_encode_fh(struct dentry *dentry, struct fid *fid,
	int *max_len, int connectable);
extern struct dentry *exportfs_decode_fh(struct vfsmount *mnt, struct fid *fid,
	int fh_len, int fileid_type, int (*acceptable)(void *, struct dentry *),
	void *context);

extern struct dentry *generic_fh_to_dentry(struct super_block *sb,
	struct fid *fid, int fh_len, int fh_type,
	struct inode *(*get_inode) (struct super_block *sb, u64 ino, u32 gen));
extern struct dentry *generic_fh_to_parent(struct super_block *sb,
	struct fid *fid, int fh_len, int fh_type,
	struct inode *(*get_inode) (struct super_block *sb, u64 ino, u32 gen));

#endif /* LINUX_EXPORTFS_H */
