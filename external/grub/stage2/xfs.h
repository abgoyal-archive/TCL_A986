/* xfs.h - an extraction from xfsprogs-1.3.5/include/xfs* into one file */

typedef signed char	xfs_int8_t;
typedef unsigned char	xfs_uint8_t;
typedef short		xfs_int16_t;
typedef unsigned short	xfs_uint16_t;
typedef int		xfs_int32_t;
typedef unsigned int	xfs_uint32_t;
typedef long long	xfs_int64_t;
typedef unsigned long long xfs_uint64_t;

typedef xfs_uint64_t	xfs_ino_t;
typedef	xfs_uint32_t	xfs_agino_t;
typedef xfs_int64_t	xfs_daddr_t;
typedef xfs_int64_t	xfs_off_t;
typedef xfs_uint8_t	uuid_t[16];


/* those are from xfs_types.h */

typedef xfs_uint32_t	xfs_agblock_t;	/* blockno in alloc. group */
typedef	xfs_uint32_t	xfs_extlen_t;	/* extent length in blocks */
typedef	xfs_uint32_t	xfs_agnumber_t;	/* allocation group number */
typedef xfs_int32_t	xfs_extnum_t;	/* # of extents in a file */
typedef xfs_int16_t	xfs_aextnum_t;	/* # extents in an attribute fork */
typedef	xfs_int64_t	xfs_fsize_t;	/* bytes in a file */

typedef	xfs_uint32_t	xfs_dablk_t;	/* dir/attr block number (in file) */
typedef	xfs_uint32_t	xfs_dahash_t;	/* dir/attr hash value */

typedef xfs_uint64_t	xfs_dfsbno_t;	/* blockno in filesystem (agno|agbno) */
typedef xfs_uint64_t	xfs_drfsbno_t;	/* blockno in filesystem (raw) */
typedef	xfs_uint64_t	xfs_drtbno_t;	/* extent (block) in realtime area */
typedef	xfs_uint64_t	xfs_dfiloff_t;	/* block number in a file */

typedef	xfs_uint64_t	xfs_fsblock_t;	/* blockno in filesystem (agno|agbno) */
typedef	xfs_uint64_t	xfs_fileoff_t;	/* block number in a file */
typedef	xfs_uint64_t	xfs_filblks_t;	/* number of blocks in a file */


/* those are from xfs_sb.h */

#define	XFS_SB_MAGIC		0x58465342	/* 'XFSB'*/
#define	XFS_SB_VERSION_4	4		/* 6.2+ - bitmask version */
#define	XFS_SB_VERSION_NUMBITS	0x000f

typedef struct xfs_sb
{
	xfs_uint32_t	sb_magicnum;	/* magic number == XFS_SB_MAGIC */
	xfs_uint32_t	sb_blocksize;	/* logical block size, bytes */
	xfs_drfsbno_t	sb_dblocks;	/* number of data blocks */
	xfs_drfsbno_t	sb_rblocks;	/* number of realtime blocks */
	xfs_drtbno_t	sb_rextents;	/* number of realtime extents */
	uuid_t		sb_uuid;	/* file system unique id */
	xfs_dfsbno_t	sb_logstart;	/* starting block of log if internal */
	xfs_ino_t	sb_rootino;	/* root inode number */
	xfs_ino_t	sb_rbmino;	/* bitmap inode for realtime extents */
	xfs_ino_t	sb_rsumino;	/* summary inode for rt bitmap */
	xfs_agblock_t	sb_rextsize;	/* realtime extent size, blocks */
	xfs_agblock_t	sb_agblocks;	/* size of an allocation group */
	xfs_agnumber_t	sb_agcount;	/* number of allocation groups */
	xfs_extlen_t	sb_rbmblocks;	/* number of rt bitmap blocks */
	xfs_extlen_t	sb_logblocks;	/* number of log blocks */
	xfs_uint16_t	sb_versionnum;	/* header version == XFS_SB_VERSION */
	xfs_uint16_t	sb_sectsize;	/* volume sector size, bytes */
	xfs_uint16_t	sb_inodesize;	/* inode size, bytes */
	xfs_uint16_t	sb_inopblock;	/* inodes per block */
	char		sb_fname[12];	/* file system name */
	xfs_uint8_t	sb_blocklog;	/* log2 of sb_blocksize */
	xfs_uint8_t	sb_sectlog;	/* log2 of sb_sectsize */
	xfs_uint8_t	sb_inodelog;	/* log2 of sb_inodesize */
	xfs_uint8_t	sb_inopblog;	/* log2 of sb_inopblock */
	xfs_uint8_t	sb_agblklog;	/* log2 of sb_agblocks (rounded up) */
	xfs_uint8_t	sb_rextslog;	/* log2 of sb_rextents */
	xfs_uint8_t	sb_inprogress;	/* mkfs is in progress, don't mount */
	xfs_uint8_t	sb_imax_pct;	/* max % of fs for inode space */
					/* statistics */
	/*
	 * These fields must remain contiguous.  If you really
	 * want to change their layout, make sure you fix the
	 * code in xfs_trans_apply_sb_deltas().
	 */
	xfs_uint64_t	sb_icount;	/* allocated inodes */
	xfs_uint64_t	sb_ifree;	/* free inodes */
	xfs_uint64_t	sb_fdblocks;	/* free data blocks */
	xfs_uint64_t	sb_frextents;	/* free realtime extents */
	/*
	 * End contiguous fields.
	 */
	xfs_ino_t	sb_uquotino;	/* user quota inode */
	xfs_ino_t	sb_gquotino;	/* group quota inode */
	xfs_uint16_t	sb_qflags;	/* quota flags */
	xfs_uint8_t	sb_flags;	/* misc. flags */
	xfs_uint8_t	sb_shared_vn;	/* shared version number */
	xfs_extlen_t	sb_inoalignmt;	/* inode chunk alignment, fsblocks */
	xfs_uint32_t	sb_unit;	/* stripe or raid unit */
	xfs_uint32_t	sb_width;	/* stripe or raid width */	
	xfs_uint8_t	sb_dirblklog;	/* log2 of dir block size (fsbs) */
        xfs_uint8_t       sb_dummy[7];    /* padding */
} xfs_sb_t;


/* those are from xfs_btree.h */

typedef struct xfs_btree_lblock
{
	xfs_uint32_t	bb_magic;	/* magic number for block type */
	xfs_uint16_t	bb_level;	/* 0 is a leaf */
	xfs_uint16_t	bb_numrecs;	/* current # of data records */
	xfs_dfsbno_t	bb_leftsib;	/* left sibling block or NULLDFSBNO */
	xfs_dfsbno_t	bb_rightsib;	/* right sibling block or NULLDFSBNO */
} xfs_btree_lblock_t;

typedef struct xfs_btree_hdr
{
	xfs_uint32_t	bb_magic;	/* magic number for block type */
	xfs_uint16_t	bb_level;	/* 0 is a leaf */
	xfs_uint16_t	bb_numrecs;	/* current # of data records */
} xfs_btree_hdr_t;

typedef struct xfs_btree_block
{
	xfs_btree_hdr_t	bb_h;		/* header */
	union		{
		struct	{
			xfs_agblock_t	bb_leftsib;
			xfs_agblock_t	bb_rightsib;
		}	s;		/* short form pointers */
		struct	{
			xfs_dfsbno_t	bb_leftsib;
			xfs_dfsbno_t	bb_rightsib;
		}	l;		/* long form pointers */
	}		bb_u;		/* rest */
} xfs_btree_block_t;

/* those are from xfs_bmap_btree.h */

typedef struct xfs_bmdr_block
{
	xfs_uint16_t	bb_level;	/* 0 is a leaf */
	xfs_uint16_t	bb_numrecs;	/* current # of data records */
} xfs_bmdr_block_t;


#define	BMBT_USE_64	1

typedef struct xfs_bmbt_rec_32
{
	xfs_uint32_t		l0, l1, l2, l3;
} xfs_bmbt_rec_32_t;
typedef struct xfs_bmbt_rec_64
{
	xfs_uint64_t		l0, l1;
} xfs_bmbt_rec_64_t;

#if BMBT_USE_64
typedef	xfs_uint64_t	xfs_bmbt_rec_base_t;	/* use this for casts */
typedef xfs_bmbt_rec_64_t xfs_bmbt_rec_t, xfs_bmdr_rec_t;
#else	/* !BMBT_USE_64 */
typedef	xfs_uint32_t	xfs_bmbt_rec_base_t;	/* use this for casts */
typedef xfs_bmbt_rec_32_t xfs_bmbt_rec_t, xfs_bmdr_rec_t;
#endif	/* BMBT_USE_64 */

typedef struct xfs_bmbt_key
{
	xfs_dfiloff_t	br_startoff;	/* starting file offset */
} xfs_bmbt_key_t, xfs_bmdr_key_t;

typedef xfs_dfsbno_t xfs_bmbt_ptr_t, xfs_bmdr_ptr_t;	/* btree pointer type */
					/* btree block header type */
typedef	struct xfs_btree_lblock xfs_bmbt_block_t;


/* those are from xfs_dir2.h */

typedef	xfs_uint16_t	xfs_dir2_data_off_t;

typedef	xfs_off_t		xfs_dir2_off_t;

/* those are from xfs_da_btree.h */

#define	XFS_DIR2_LEAF1_MAGIC	0xd2f1	/* magic number: v2 dirlf single blks */
#define	XFS_DIR2_LEAFN_MAGIC	0xd2ff	/* magic number: v2 dirlf multi blks */

typedef struct xfs_da_blkinfo {
	xfs_dablk_t forw;			/* previous block in list */
	xfs_dablk_t back;			/* following block in list */
	xfs_uint16_t magic;			/* validity check on block */
	xfs_uint16_t pad;				/* unused */
} xfs_da_blkinfo_t;


typedef struct xfs_da_intnode {
	struct xfs_da_node_hdr {	/* constant-structure header block */
		xfs_da_blkinfo_t info;	/* block type, links, etc. */
		xfs_uint16_t count;	/* count of active entries */
		xfs_uint16_t level;	/* level above leaves (leaf == 0) */
	} hdr;
	struct xfs_da_node_entry {
		xfs_dahash_t hashval;	/* hash value for this descendant */
		xfs_dablk_t before;	/* Btree block before this key */
	} btree[1];			/* variable sized array of keys */
} xfs_da_intnode_t;


/* those are from xfs_dir2_data.h */

#define	XFS_DIR2_DATA_FREE_TAG	0xffff
#define	XFS_DIR2_DATA_FD_COUNT	3


typedef struct xfs_dir2_data_free {
	xfs_dir2_data_off_t	offset;		/* start of freespace */
	xfs_dir2_data_off_t	length;		/* length of freespace */
} xfs_dir2_data_free_t;

typedef struct xfs_dir2_data_hdr {
	xfs_uint32_t		magic;		/* XFS_DIR2_DATA_MAGIC */
						/* or XFS_DIR2_BLOCK_MAGIC */
	xfs_dir2_data_free_t	bestfree[XFS_DIR2_DATA_FD_COUNT];
} xfs_dir2_data_hdr_t;

typedef struct xfs_dir2_data_entry {
	xfs_ino_t		inumber;	/* inode number */
	xfs_uint8_t		namelen;	/* name length */
	xfs_uint8_t		name[1];	/* name bytes, no null */
						/* variable offset */
	xfs_dir2_data_off_t	tag;		/* starting offset of us */
} xfs_dir2_data_entry_t;

typedef struct xfs_dir2_data_unused {
	xfs_uint16_t		freetag;	/* XFS_DIR2_DATA_FREE_TAG */
	xfs_dir2_data_off_t	length;		/* total free length */
						/* variable offset */
	xfs_dir2_data_off_t	tag;		/* starting offset of us */
} xfs_dir2_data_unused_t;

typedef union {
	xfs_dir2_data_entry_t	entry;
	xfs_dir2_data_unused_t	unused;
} xfs_dir2_data_union_t;


/* those are from xfs_dir2_leaf.h */

typedef struct xfs_dir2_leaf_hdr {
	xfs_da_blkinfo_t	info;		/* header for da routines */
	xfs_uint16_t		count;		/* count of entries */
	xfs_uint16_t		stale;		/* count of stale entries */
} xfs_dir2_leaf_hdr_t;


/* those are from xfs_dir2_block.h */


#define	XFS_DIR2_BLOCK_MAGIC	0x58443242	/* XD2B: for one block dirs */

typedef struct xfs_dir2_block_tail {
	xfs_uint32_t	count;			/* count of leaf entries */
	xfs_uint32_t	stale;			/* count of stale lf entries */
} xfs_dir2_block_tail_t;


/* those are from xfs_dir2_sf.h */


typedef	struct { xfs_uint8_t i[8]; } xfs_dir2_ino8_t;

typedef struct { xfs_uint8_t i[4]; } xfs_dir2_ino4_t;

typedef union {
	xfs_dir2_ino8_t	i8;
	xfs_dir2_ino4_t	i4;
} xfs_dir2_inou_t;

typedef struct { xfs_uint8_t i[2]; } xfs_dir2_sf_off_t;

typedef struct xfs_dir2_sf_hdr {
	xfs_uint8_t		count;		/* count of entries */
	xfs_uint8_t		i8count;	/* count of 8-byte inode #s */
	xfs_dir2_inou_t		parent;		/* parent dir inode number */
} xfs_dir2_sf_hdr_t;

typedef struct xfs_dir2_sf_entry {
	xfs_uint8_t		namelen;	/* actual name length */
	xfs_dir2_sf_off_t	offset;		/* saved offset */
	xfs_uint8_t		name[1];	/* name, variable size */
	xfs_dir2_inou_t		inumber;	/* inode number, var. offset */
} xfs_dir2_sf_entry_t;

typedef struct xfs_dir2_sf {
	xfs_dir2_sf_hdr_t	hdr;		/* shortform header */
	xfs_dir2_sf_entry_t	list[1];	/* shortform entries */
} xfs_dir2_sf_t;

/* those are from xfs_dinode.h */

#define	XFS_DINODE_VERSION_1	1
#define	XFS_DINODE_VERSION_2	2
#define	XFS_DINODE_MAGIC	0x494e	/* 'IN' */

typedef struct xfs_timestamp {
	xfs_int32_t	t_sec;		/* timestamp seconds */
	xfs_int32_t	t_nsec;		/* timestamp nanoseconds */
} xfs_timestamp_t;

typedef struct xfs_dinode_core
{
	xfs_uint16_t	di_magic;	/* inode magic # = XFS_DINODE_MAGIC */
	xfs_uint16_t	di_mode;	/* mode and type of file */
	xfs_int8_t	di_version;	/* inode version */
	xfs_int8_t	di_format;	/* format of di_c data */
	xfs_uint16_t	di_onlink;	/* old number of links to file */
	xfs_uint32_t	di_uid;		/* owner's user id */
	xfs_uint32_t	di_gid;		/* owner's group id */
	xfs_uint32_t	di_nlink;	/* number of links to file */
	xfs_uint16_t	di_projid;	/* owner's project id */
	xfs_uint8_t	di_pad[10];	/* unused, zeroed space */
	xfs_timestamp_t	di_atime;	/* time last accessed */
	xfs_timestamp_t	di_mtime;	/* time last modified */
	xfs_timestamp_t	di_ctime;	/* time created/inode modified */
	xfs_fsize_t	di_size;	/* number of bytes in file */
	xfs_drfsbno_t	di_nblocks;	/* # of direct & btree blocks used */
	xfs_extlen_t	di_extsize;	/* basic/minimum extent size for file */
	xfs_extnum_t	di_nextents;	/* number of extents in data fork */
	xfs_aextnum_t	di_anextents;	/* number of extents in attribute fork*/
	xfs_uint8_t	di_forkoff;	/* attr fork offs, <<3 for 64b align */
	xfs_int8_t	di_aformat;	/* format of attr fork's data */
	xfs_uint32_t	di_dmevmask;	/* DMIG event mask */
	xfs_uint16_t	di_dmstate;	/* DMIG state info */
	xfs_uint16_t	di_flags;	/* random flags, XFS_DIFLAG_... */
	xfs_uint32_t	di_gen;		/* generation number */
} xfs_dinode_core_t;

typedef struct xfs_dinode
{
	xfs_dinode_core_t	di_core;
	xfs_agino_t		di_next_unlinked;/* agi unlinked list ptr */
	union {
		xfs_bmdr_block_t di_bmbt;	/* btree root block */
		xfs_bmbt_rec_32_t di_bmx[1];	/* extent list */
		xfs_dir2_sf_t	di_dir2sf;	/* shortform directory v2 */
		char		di_c[1];	/* local contents */
	} di_u;
} xfs_dinode_t;

typedef enum xfs_dinode_fmt
{
	XFS_DINODE_FMT_DEV,		/* CHR, BLK: di_dev */
	XFS_DINODE_FMT_LOCAL,		/* DIR, REG: di_c */
					/* LNK: di_symlink */
	XFS_DINODE_FMT_EXTENTS,		/* DIR, REG, LNK: di_bmx */
	XFS_DINODE_FMT_BTREE,		/* DIR, REG, LNK: di_bmbt */
	XFS_DINODE_FMT_UUID 		/* MNT: di_uuid */
} xfs_dinode_fmt_t;

#define	IFMT		0170000		/* type of file */
#define	IFDIR		0040000		/* directory */
#define	IFREG		0100000		/* regular */
#define	IFLNK		0120000		/* symbolic link */
