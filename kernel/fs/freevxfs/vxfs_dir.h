
#ifndef _VXFS_DIR_H_
#define _VXFS_DIR_H_



struct vxfs_dirblk {
	u_int16_t	d_free;		/* free space in dirblock */
	u_int16_t	d_nhash;	/* no of hash chains */
	u_int16_t	d_hash[1];	/* hash chain */
};

#define VXFS_NAMELEN	256

struct vxfs_direct {
	vx_ino_t	d_ino;			/* inode number */
	u_int16_t	d_reclen;		/* record length */
	u_int16_t	d_namelen;		/* d_name length */
	u_int16_t	d_hashnext;		/* next hash entry */
	char		d_name[VXFS_NAMELEN];	/* name */
};

#define VXFS_DIRPAD		4
#define VXFS_NAMEMIN		offsetof(struct vxfs_direct, d_name)
#define VXFS_DIRROUND(len)	((VXFS_DIRPAD + (len) - 1) & ~(VXFS_DIRPAD -1))
#define VXFS_DIRLEN(len)	(VXFS_DIRROUND(VXFS_NAMEMIN + (len)))

#define VXFS_DIRBLKOV(dbp)	((sizeof(short) * dbp->d_nhash) + 4)

#endif /* _VXFS_DIR_H_ */
