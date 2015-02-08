
#ifndef _H_JFS_ACL
#define _H_JFS_ACL

#ifdef CONFIG_JFS_POSIX_ACL

int jfs_check_acl(struct inode *, int);
int jfs_init_acl(tid_t, struct inode *, struct inode *);
int jfs_acl_chmod(struct inode *inode);

#else

static inline int jfs_init_acl(tid_t tid, struct inode *inode,
			       struct inode *dir)
{
	return 0;
}

static inline int jfs_acl_chmod(struct inode *inode)
{
	return 0;
}

#endif
#endif		/* _H_JFS_ACL */
