
#ifndef _VXFS_INODE_H_
#define _VXFS_INODE_H_



#define VXFS_ISIZE		0x100		/* Inode size */

#define VXFS_NDADDR		10		/* Number of direct addrs in inode */
#define VXFS_NIADDR		2		/* Number of indirect addrs in inode */
#define VXFS_NIMMED		96		/* Size of immediate data in inode */
#define VXFS_NTYPED		6		/* Num of typed extents */

#define VXFS_TYPED_OFFSETMASK	(0x00FFFFFFFFFFFFFFULL)
#define VXFS_TYPED_TYPEMASK	(0xFF00000000000000ULL)
#define VXFS_TYPED_TYPESHIFT	56

#define VXFS_TYPED_PER_BLOCK(sbp) \
	((sbp)->s_blocksize / sizeof(struct vxfs_typed))

enum {
	VXFS_TYPED_INDIRECT		= 1,
	VXFS_TYPED_DATA			= 2,
	VXFS_TYPED_INDIRECT_DEV4	= 3,
	VXFS_TYPED_DATA_DEV4		= 4,
};

struct vxfs_immed {
	u_int8_t	vi_immed[VXFS_NIMMED];
};

struct vxfs_ext4 {
	u_int32_t		ve4_spare;		/* ?? */
	u_int32_t		ve4_indsize;		/* Indirect extent size */
	vx_daddr_t		ve4_indir[VXFS_NIADDR];	/* Indirect extents */
	struct direct {					/* Direct extents */
		vx_daddr_t	extent;			/* Extent number */
		int32_t		size;			/* Size of extent */
	} ve4_direct[VXFS_NDADDR];
};

struct vxfs_typed {
	u_int64_t	vt_hdr;		/* Header, 0xTTOOOOOOOOOOOOOO; T=type,O=offs */
	vx_daddr_t	vt_block;	/* Extent block */
	int32_t		vt_size;	/* Size in blocks */
};

struct vxfs_typed_dev4 {
	u_int64_t	vd4_hdr;	/* Header, 0xTTOOOOOOOOOOOOOO; T=type,O=offs */
	u_int64_t	vd4_block;	/* Extent block */
	u_int64_t	vd4_size;	/* Size in blocks */
	int32_t		vd4_dev;	/* Device ID */
	u_int32_t	__pad1;
};

struct vxfs_dinode {
	int32_t		vdi_mode;
	u_int32_t	vdi_nlink;	/* Link count */
	u_int32_t	vdi_uid;	/* UID */
	u_int32_t	vdi_gid;	/* GID */
	u_int64_t	vdi_size;	/* Inode size in bytes */
	u_int32_t	vdi_atime;	/* Last time accessed - sec */
	u_int32_t	vdi_autime;	/* Last time accessed - usec */
	u_int32_t	vdi_mtime;	/* Last modify time - sec */
	u_int32_t	vdi_mutime;	/* Last modify time - usec */
	u_int32_t	vdi_ctime;	/* Create time - sec */
	u_int32_t	vdi_cutime;	/* Create time - usec */
	u_int8_t	vdi_aflags;	/* Allocation flags */
	u_int8_t	vdi_orgtype;	/* Organisation type */
	u_int16_t	vdi_eopflags;
	u_int32_t	vdi_eopdata;
	union {
		u_int32_t		rdev;
		u_int32_t		dotdot;
		struct {
			u_int32_t	reserved;
			u_int32_t	fixextsize;
		} i_regular;
		struct {
			u_int32_t	matchino;
			u_int32_t	fsetindex;
		} i_vxspec;
		u_int64_t		align;
	} vdi_ftarea;
	u_int32_t	vdi_blocks;	/* How much blocks does inode occupy */
	u_int32_t	vdi_gen;	/* Inode generation */
	u_int64_t	vdi_version;	/* Version */
	union {
		struct vxfs_immed	immed;
		struct vxfs_ext4	ext4;
		struct vxfs_typed	typed[VXFS_NTYPED];
	} vdi_org;
	u_int32_t	vdi_iattrino;
};

#define vdi_rdev	vdi_ftarea.rdev
#define vdi_dotdot	vdi_ftarea.dotdot
#define vdi_fixextsize	vdi_ftarea.regular.fixextsize
#define vdi_matchino	vdi_ftarea.vxspec.matchino
#define vdi_fsetindex	vdi_ftarea.vxspec.fsetindex

#define vdi_immed	vdi_org.immed
#define vdi_ext4	vdi_org.ext4
#define vdi_typed	vdi_org.typed


#define vxfs_inode_info	vxfs_dinode

#define vii_mode	vdi_mode
#define vii_uid		vdi_uid
#define vii_gid		vdi_gid
#define vii_nlink	vdi_nlink
#define vii_size	vdi_size
#define vii_atime	vdi_atime
#define vii_ctime	vdi_ctime
#define vii_mtime	vdi_mtime
#define vii_blocks	vdi_blocks
#define vii_org		vdi_org
#define vii_orgtype	vdi_orgtype
#define vii_gen		vdi_gen

#define vii_rdev	vdi_ftarea.rdev
#define vii_dotdot	vdi_ftarea.dotdot
#define vii_fixextsize	vdi_ftarea.regular.fixextsize
#define vii_matchino	vdi_ftarea.vxspec.matchino
#define vii_fsetindex	vdi_ftarea.vxspec.fsetindex

#define vii_immed	vdi_org.immed
#define vii_ext4	vdi_org.ext4
#define vii_typed	vdi_org.typed

#endif /* _VXFS_INODE_H_ */
