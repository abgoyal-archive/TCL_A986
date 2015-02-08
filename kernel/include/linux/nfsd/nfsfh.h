

#ifndef _LINUX_NFSD_FH_H
#define _LINUX_NFSD_FH_H

# include <linux/types.h>
#ifdef __KERNEL__
# include <linux/sunrpc/svc.h>
#endif
#include <linux/nfsd/const.h>

struct nfs_fhbase_old {
	__u32		fb_dcookie;	/* dentry cookie - always 0xfeebbaca */
	__u32		fb_ino;		/* our inode number */
	__u32		fb_dirino;	/* dir inode number, 0 for directories */
	__u32		fb_dev;		/* our device */
	__u32		fb_xdev;
	__u32		fb_xino;
	__u32		fb_generation;
};

struct nfs_fhbase_new {
	__u8		fb_version;	/* == 1, even => nfs_fhbase_old */
	__u8		fb_auth_type;
	__u8		fb_fsid_type;
	__u8		fb_fileid_type;
	__u32		fb_auth[1];
/*	__u32		fb_fsid[0]; floating */
/*	__u32		fb_fileid[0]; floating */
};

struct knfsd_fh {
	unsigned int	fh_size;	/* significant for NFSv3.
					 * Points to the current size while building
					 * a new file handle
					 */
	union {
		struct nfs_fhbase_old	fh_old;
		__u32			fh_pad[NFS4_FHSIZE/4];
		struct nfs_fhbase_new	fh_new;
	} fh_base;
};

#define ofh_dcookie		fh_base.fh_old.fb_dcookie
#define ofh_ino			fh_base.fh_old.fb_ino
#define ofh_dirino		fh_base.fh_old.fb_dirino
#define ofh_dev			fh_base.fh_old.fb_dev
#define ofh_xdev		fh_base.fh_old.fb_xdev
#define ofh_xino		fh_base.fh_old.fb_xino
#define ofh_generation		fh_base.fh_old.fb_generation

#define	fh_version		fh_base.fh_new.fb_version
#define	fh_fsid_type		fh_base.fh_new.fb_fsid_type
#define	fh_auth_type		fh_base.fh_new.fb_auth_type
#define	fh_fileid_type		fh_base.fh_new.fb_fileid_type
#define	fh_auth			fh_base.fh_new.fb_auth
#define	fh_fsid			fh_base.fh_new.fb_auth

#ifdef __KERNEL__

static inline __u32 ino_t_to_u32(ino_t ino)
{
	return (__u32) ino;
}

static inline ino_t u32_to_ino_t(__u32 uino)
{
	return (ino_t) uino;
}

typedef struct svc_fh {
	struct knfsd_fh		fh_handle;	/* FH data */
	struct dentry *		fh_dentry;	/* validated dentry */
	struct svc_export *	fh_export;	/* export pointer */
	int			fh_maxsize;	/* max size for fh_handle */

	unsigned char		fh_locked;	/* inode locked by us */

#ifdef CONFIG_NFSD_V3
	unsigned char		fh_post_saved;	/* post-op attrs saved */
	unsigned char		fh_pre_saved;	/* pre-op attrs saved */

	/* Pre-op attributes saved during fh_lock */
	__u64			fh_pre_size;	/* size before operation */
	struct timespec		fh_pre_mtime;	/* mtime before oper */
	struct timespec		fh_pre_ctime;	/* ctime before oper */
	/*
	 * pre-op nfsv4 change attr: note must check IS_I_VERSION(inode)
	 *  to find out if it is valid.
	 */
	u64			fh_pre_change;

	/* Post-op attributes saved in fh_unlock */
	struct kstat		fh_post_attr;	/* full attrs after operation */
	u64			fh_post_change; /* nfsv4 change; see above */
#endif /* CONFIG_NFSD_V3 */

} svc_fh;

#endif /* __KERNEL__ */


#endif /* _LINUX_NFSD_FH_H */
