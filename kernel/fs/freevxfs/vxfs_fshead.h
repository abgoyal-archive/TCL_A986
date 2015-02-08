
#ifndef _VXFS_FSHEAD_H_
#define _VXFS_FSHEAD_H_



struct vxfs_fsh {
	u_int32_t	fsh_version;		/* fileset header version */
	u_int32_t	fsh_fsindex;		/* fileset index */
	u_int32_t	fsh_time;		/* modification time - sec */
	u_int32_t	fsh_utime;		/* modification time - usec */
	u_int32_t	fsh_extop;		/* extop flags */
	vx_ino_t	fsh_ninodes;		/* allocated inodes */
	u_int32_t	fsh_nau;		/* number of IAUs */
	u_int32_t	fsh_old_ilesize;	/* old size of ilist */
	u_int32_t	fsh_dflags;		/* flags */
	u_int32_t	fsh_quota;		/* quota limit */
	vx_ino_t	fsh_maxinode;		/* maximum inode number */
	vx_ino_t	fsh_iauino;		/* IAU inode */
	vx_ino_t	fsh_ilistino[2];	/* ilist inodes */
	vx_ino_t	fsh_lctino;		/* link count table inode */

	/*
	 * Slightly more fields follow, but they
	 *  a) are not of any interest for us, and
	 *  b) differ a lot in different vxfs versions/ports
	 */
};

#endif /* _VXFS_FSHEAD_H_ */
