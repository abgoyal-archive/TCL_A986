

struct super_block;
struct linux_binprm;
struct path;

#ifdef CONFIG_BLOCK
extern struct super_block *blockdev_superblock;
extern void __init bdev_cache_init(void);

static inline int sb_is_blkdev_sb(struct super_block *sb)
{
	return sb == blockdev_superblock;
}

extern int __sync_blockdev(struct block_device *bdev, int wait);

#else
static inline void bdev_cache_init(void)
{
}

static inline int sb_is_blkdev_sb(struct super_block *sb)
{
	return 0;
}

static inline int __sync_blockdev(struct block_device *bdev, int wait)
{
	return 0;
}
#endif

extern void __init chrdev_init(void);

extern int check_unsafe_exec(struct linux_binprm *);

extern int copy_mount_options(const void __user *, unsigned long *);
extern int copy_mount_string(const void __user *, char **);

extern void free_vfsmnt(struct vfsmount *);
extern struct vfsmount *alloc_vfsmnt(const char *);
extern struct vfsmount *__lookup_mnt(struct vfsmount *, struct dentry *, int);
extern void mnt_set_mountpoint(struct vfsmount *, struct dentry *,
				struct vfsmount *);
extern void release_mounts(struct list_head *);
extern void umount_tree(struct vfsmount *, int, struct list_head *);
extern struct vfsmount *copy_tree(struct vfsmount *, struct dentry *, int);

extern void __init mnt_init(void);

extern spinlock_t vfsmount_lock;

extern void chroot_fs_refs(struct path *, struct path *);

extern void mark_files_ro(struct super_block *);
extern struct file *get_empty_filp(void);

extern int do_remount_sb(struct super_block *, int, void *, int);
extern void __put_super(struct super_block *sb);
extern void put_super(struct super_block *sb);

struct nameidata;
extern struct file *nameidata_to_filp(struct nameidata *);
extern void release_open_intent(struct nameidata *);
