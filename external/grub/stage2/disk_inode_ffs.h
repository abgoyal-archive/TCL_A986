
#ifndef	_BOOT_UFS_DISK_INODE_FFS_H_
#define	_BOOT_UFS_DISK_INODE_FFS_H_

#define	NDADDR	FFS_NDADDR
#define	NIADDR	FFS_NIADDR

#define	MAX_FASTLINK_SIZE	FFS_MAX_FASTLINK_SIZE

#define	IC_FASTLINK	0x0001	/* Symbolic link in inode */

#define	i_mode		ic_mode
#define	i_nlink		ic_nlink
#define	i_uid		ic_uid
#define	i_gid		ic_gid
#if	defined(BYTE_MSF) && BYTE_MSF
#define	i_size		ic_size.val[1]
#else /* BYTE_LSF */
#define	i_size		ic_size.val[0]
#endif
#define	i_db		ic_db
#define	i_ib		ic_ib
#define	i_atime		ic_atime
#define	i_mtime		ic_mtime
#define	i_ctime		ic_ctime
#define i_blocks	ic_blocks
#define	i_rdev		ic_db[0]
#define	i_symlink	ic_symlink
#define i_flags		ic_flags
#define i_gen		ic_gen

/* modes */
#define	IFMT	0xf000		/* type of file */
#define	IFCHR	0x2000		/* character special */
#define	IFDIR	0x4000		/* directory */
#define	IFBLK	0x6000		/* block special */
#define	IFREG	0x8000		/* regular */
#define	IFLNK	0xa000		/* symbolic link */
#define	IFSOCK	0xc000		/* socket */


#define	ISUID		0x0800	/* set user id on execution */
#define	ISGID		0x0400	/* set group id on execution */
#define	ISVTX		0x0200	/* save swapped text even after use */
#define	IREAD		0x0100	/* read, write, execute permissions */
#define	IWRITE		0x0080
#define	IEXEC		0x0040

#ifdef EEK
#define f_fs		u.ffs.ffs_fs
#define i_ic		u.ffs.ffs_ic
#define f_nindir	u.ffs.ffs_nindir
#define f_blk		u.ffs.ffs_blk
#define f_blksize	u.ffs.ffs_blksize
#define f_blkno		u.ffs.ffs_blkno
#endif /* EEK */

#endif	/* _BOOT_UFS_DISK_INODE_FFS_H_ */
