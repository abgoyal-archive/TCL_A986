


#ifndef __UBIFS_MEDIA_H__
#define __UBIFS_MEDIA_H__

/* UBIFS node magic number (must not have the padding byte first or last) */
#define UBIFS_NODE_MAGIC  0x06101831

#define UBIFS_FORMAT_VERSION 4

#define UBIFS_RO_COMPAT_VERSION 0

/* Minimum logical eraseblock size in bytes */
#define UBIFS_MIN_LEB_SZ (15*1024)

/* Initial CRC32 value used when calculating CRC checksums */
#define UBIFS_CRC32_INIT 0xFFFFFFFFU

#define UBIFS_MIN_COMPR_LEN 128

#define UBIFS_MIN_COMPRESS_DIFF 64

/* Root inode number */
#define UBIFS_ROOT_INO 1

/* Lowest inode number used for regular inodes (not UBIFS-only internal ones) */
#define UBIFS_FIRST_INO 64

#define UBIFS_MAX_NLEN 255

/* Maximum number of data journal heads */
#define UBIFS_MAX_JHEADS 1

#define UBIFS_BLOCK_SIZE  4096
#define UBIFS_BLOCK_SHIFT 12

/* UBIFS padding byte pattern (must not be first or last byte of node magic) */
#define UBIFS_PADDING_BYTE 0xCE

/* Maximum possible key length */
#define UBIFS_MAX_KEY_LEN 16

/* Key length ("simple" format) */
#define UBIFS_SK_LEN 8

/* Minimum index tree fanout */
#define UBIFS_MIN_FANOUT 3

/* Maximum number of levels in UBIFS indexing B-tree */
#define UBIFS_MAX_LEVELS 512

/* Maximum amount of data attached to an inode in bytes */
#define UBIFS_MAX_INO_DATA UBIFS_BLOCK_SIZE

/* LEB Properties Tree fanout (must be power of 2) and fanout shift */
#define UBIFS_LPT_FANOUT 4
#define UBIFS_LPT_FANOUT_SHIFT 2

/* LEB Properties Tree bit field sizes */
#define UBIFS_LPT_CRC_BITS 16
#define UBIFS_LPT_CRC_BYTES 2
#define UBIFS_LPT_TYPE_BITS 4

/* The key is always at the same position in all keyed nodes */
#define UBIFS_KEY_OFFSET offsetof(struct ubifs_ino_node, key)

/* Garbage collector journal head number */
#define UBIFS_GC_HEAD   0
/* Base journal head number */
#define UBIFS_BASE_HEAD 1
/* Data journal head number */
#define UBIFS_DATA_HEAD 2

enum {
	UBIFS_LPT_PNODE,
	UBIFS_LPT_NNODE,
	UBIFS_LPT_LTAB,
	UBIFS_LPT_LSAVE,
	UBIFS_LPT_NODE_CNT,
	UBIFS_LPT_NOT_A_NODE = (1 << UBIFS_LPT_TYPE_BITS) - 1,
};

enum {
	UBIFS_ITYPE_REG,
	UBIFS_ITYPE_DIR,
	UBIFS_ITYPE_LNK,
	UBIFS_ITYPE_BLK,
	UBIFS_ITYPE_CHR,
	UBIFS_ITYPE_FIFO,
	UBIFS_ITYPE_SOCK,
	UBIFS_ITYPES_CNT,
};

enum {
	UBIFS_KEY_HASH_R5,
	UBIFS_KEY_HASH_TEST,
};

enum {
	UBIFS_SIMPLE_KEY_FMT,
};

#define UBIFS_S_KEY_BLOCK_BITS 29
#define UBIFS_S_KEY_BLOCK_MASK 0x1FFFFFFF
#define UBIFS_S_KEY_HASH_BITS  UBIFS_S_KEY_BLOCK_BITS
#define UBIFS_S_KEY_HASH_MASK  UBIFS_S_KEY_BLOCK_MASK

enum {
	UBIFS_INO_KEY,
	UBIFS_DATA_KEY,
	UBIFS_DENT_KEY,
	UBIFS_XENT_KEY,
	UBIFS_KEY_TYPES_CNT,
};

/* Count of LEBs reserved for the superblock area */
#define UBIFS_SB_LEBS 1
/* Count of LEBs reserved for the master area */
#define UBIFS_MST_LEBS 2

/* First LEB of the superblock area */
#define UBIFS_SB_LNUM 0
/* First LEB of the master area */
#define UBIFS_MST_LNUM (UBIFS_SB_LNUM + UBIFS_SB_LEBS)
/* First LEB of the log area */
#define UBIFS_LOG_LNUM (UBIFS_MST_LNUM + UBIFS_MST_LEBS)


/* Minimum number of logical eraseblocks in the log */
#define UBIFS_MIN_LOG_LEBS 2
/* Minimum number of bud logical eraseblocks (one for each head) */
#define UBIFS_MIN_BUD_LEBS 3
/* Minimum number of journal logical eraseblocks */
#define UBIFS_MIN_JNL_LEBS (UBIFS_MIN_LOG_LEBS + UBIFS_MIN_BUD_LEBS)
/* Minimum number of LPT area logical eraseblocks */
#define UBIFS_MIN_LPT_LEBS 2
/* Minimum number of orphan area logical eraseblocks */
#define UBIFS_MIN_ORPH_LEBS 1
#define UBIFS_MIN_MAIN_LEBS (UBIFS_MIN_BUD_LEBS + 6)

/* Minimum number of logical eraseblocks */
#define UBIFS_MIN_LEB_CNT (UBIFS_SB_LEBS + UBIFS_MST_LEBS + \
			   UBIFS_MIN_LOG_LEBS + UBIFS_MIN_LPT_LEBS + \
			   UBIFS_MIN_ORPH_LEBS + UBIFS_MIN_MAIN_LEBS)

/* Node sizes (N.B. these are guaranteed to be multiples of 8) */
#define UBIFS_CH_SZ        sizeof(struct ubifs_ch)
#define UBIFS_INO_NODE_SZ  sizeof(struct ubifs_ino_node)
#define UBIFS_DATA_NODE_SZ sizeof(struct ubifs_data_node)
#define UBIFS_DENT_NODE_SZ sizeof(struct ubifs_dent_node)
#define UBIFS_TRUN_NODE_SZ sizeof(struct ubifs_trun_node)
#define UBIFS_PAD_NODE_SZ  sizeof(struct ubifs_pad_node)
#define UBIFS_SB_NODE_SZ   sizeof(struct ubifs_sb_node)
#define UBIFS_MST_NODE_SZ  sizeof(struct ubifs_mst_node)
#define UBIFS_REF_NODE_SZ  sizeof(struct ubifs_ref_node)
#define UBIFS_IDX_NODE_SZ  sizeof(struct ubifs_idx_node)
#define UBIFS_CS_NODE_SZ   sizeof(struct ubifs_cs_node)
#define UBIFS_ORPH_NODE_SZ sizeof(struct ubifs_orph_node)
/* Extended attribute entry nodes are identical to directory entry nodes */
#define UBIFS_XENT_NODE_SZ UBIFS_DENT_NODE_SZ
/* Only this does not have to be multiple of 8 bytes */
#define UBIFS_BRANCH_SZ    sizeof(struct ubifs_branch)

/* Maximum node sizes (N.B. these are guaranteed to be multiples of 8) */
#define UBIFS_MAX_DATA_NODE_SZ  (UBIFS_DATA_NODE_SZ + UBIFS_BLOCK_SIZE)
#define UBIFS_MAX_INO_NODE_SZ   (UBIFS_INO_NODE_SZ + UBIFS_MAX_INO_DATA)
#define UBIFS_MAX_DENT_NODE_SZ  (UBIFS_DENT_NODE_SZ + UBIFS_MAX_NLEN + 1)
#define UBIFS_MAX_XENT_NODE_SZ  UBIFS_MAX_DENT_NODE_SZ

/* The largest UBIFS node */
#define UBIFS_MAX_NODE_SZ UBIFS_MAX_INO_NODE_SZ

enum {
	UBIFS_COMPR_FL     = 0x01,
	UBIFS_SYNC_FL      = 0x02,
	UBIFS_IMMUTABLE_FL = 0x04,
	UBIFS_APPEND_FL    = 0x08,
	UBIFS_DIRSYNC_FL   = 0x10,
	UBIFS_XATTR_FL     = 0x20,
};

/* Inode flag bits used by UBIFS */
#define UBIFS_FL_MASK 0x0000001F

enum {
	UBIFS_COMPR_NONE,
	UBIFS_COMPR_LZO,
	UBIFS_COMPR_ZLIB,
	UBIFS_COMPR_TYPES_CNT,
};

enum {
	UBIFS_INO_NODE,
	UBIFS_DATA_NODE,
	UBIFS_DENT_NODE,
	UBIFS_XENT_NODE,
	UBIFS_TRUN_NODE,
	UBIFS_PAD_NODE,
	UBIFS_SB_NODE,
	UBIFS_MST_NODE,
	UBIFS_REF_NODE,
	UBIFS_IDX_NODE,
	UBIFS_CS_NODE,
	UBIFS_ORPH_NODE,
	UBIFS_NODE_TYPES_CNT,
};

enum {
	UBIFS_MST_DIRTY = 1,
	UBIFS_MST_NO_ORPHS = 2,
	UBIFS_MST_RCVRY = 4,
};

enum {
	UBIFS_NO_NODE_GROUP = 0,
	UBIFS_IN_NODE_GROUP,
	UBIFS_LAST_OF_NODE_GROUP,
};

enum {
	UBIFS_FLG_BIGLPT = 0x02,
};

struct ubifs_ch {
	__le32 magic;
	__le32 crc;
	__le64 sqnum;
	__le32 len;
	__u8 node_type;
	__u8 group_type;
	__u8 padding[2];
} __attribute__ ((packed));

union ubifs_dev_desc {
	__le32 new;
	__le64 huge;
} __attribute__ ((packed));

struct ubifs_ino_node {
	struct ubifs_ch ch;
	__u8 key[UBIFS_MAX_KEY_LEN];
	__le64 creat_sqnum;
	__le64 size;
	__le64 atime_sec;
	__le64 ctime_sec;
	__le64 mtime_sec;
	__le32 atime_nsec;
	__le32 ctime_nsec;
	__le32 mtime_nsec;
	__le32 nlink;
	__le32 uid;
	__le32 gid;
	__le32 mode;
	__le32 flags;
	__le32 data_len;
	__le32 xattr_cnt;
	__le32 xattr_size;
	__u8 padding1[4]; /* Watch 'zero_ino_node_unused()' if changing! */
	__le32 xattr_names;
	__le16 compr_type;
	__u8 padding2[26]; /* Watch 'zero_ino_node_unused()' if changing! */
	__u8 data[];
} __attribute__ ((packed));

struct ubifs_dent_node {
	struct ubifs_ch ch;
	__u8 key[UBIFS_MAX_KEY_LEN];
	__le64 inum;
	__u8 padding1;
	__u8 type;
	__le16 nlen;
	__u8 padding2[4]; /* Watch 'zero_dent_node_unused()' if changing! */
	__u8 name[];
} __attribute__ ((packed));

struct ubifs_data_node {
	struct ubifs_ch ch;
	__u8 key[UBIFS_MAX_KEY_LEN];
	__le32 size;
	__le16 compr_type;
	__u8 padding[2]; /* Watch 'zero_data_node_unused()' if changing! */
	__u8 data[];
} __attribute__ ((packed));

struct ubifs_trun_node {
	struct ubifs_ch ch;
	__le32 inum;
	__u8 padding[12]; /* Watch 'zero_trun_node_unused()' if changing! */
	__le64 old_size;
	__le64 new_size;
} __attribute__ ((packed));

struct ubifs_pad_node {
	struct ubifs_ch ch;
	__le32 pad_len;
} __attribute__ ((packed));

struct ubifs_sb_node {
	struct ubifs_ch ch;
	__u8 padding[2];
	__u8 key_hash;
	__u8 key_fmt;
	__le32 flags;
	__le32 min_io_size;
	__le32 leb_size;
	__le32 leb_cnt;
	__le32 max_leb_cnt;
	__le64 max_bud_bytes;
	__le32 log_lebs;
	__le32 lpt_lebs;
	__le32 orph_lebs;
	__le32 jhead_cnt;
	__le32 fanout;
	__le32 lsave_cnt;
	__le32 fmt_version;
	__le16 default_compr;
	__u8 padding1[2];
	__le32 rp_uid;
	__le32 rp_gid;
	__le64 rp_size;
	__le32 time_gran;
	__u8 uuid[16];
	__le32 ro_compat_version;
	__u8 padding2[3968];
} __attribute__ ((packed));

struct ubifs_mst_node {
	struct ubifs_ch ch;
	__le64 highest_inum;
	__le64 cmt_no;
	__le32 flags;
	__le32 log_lnum;
	__le32 root_lnum;
	__le32 root_offs;
	__le32 root_len;
	__le32 gc_lnum;
	__le32 ihead_lnum;
	__le32 ihead_offs;
	__le64 index_size;
	__le64 total_free;
	__le64 total_dirty;
	__le64 total_used;
	__le64 total_dead;
	__le64 total_dark;
	__le32 lpt_lnum;
	__le32 lpt_offs;
	__le32 nhead_lnum;
	__le32 nhead_offs;
	__le32 ltab_lnum;
	__le32 ltab_offs;
	__le32 lsave_lnum;
	__le32 lsave_offs;
	__le32 lscan_lnum;
	__le32 empty_lebs;
	__le32 idx_lebs;
	__le32 leb_cnt;
	__u8 padding[344];
} __attribute__ ((packed));

struct ubifs_ref_node {
	struct ubifs_ch ch;
	__le32 lnum;
	__le32 offs;
	__le32 jhead;
	__u8 padding[28];
} __attribute__ ((packed));

struct ubifs_branch {
	__le32 lnum;
	__le32 offs;
	__le32 len;
	__u8 key[];
} __attribute__ ((packed));

struct ubifs_idx_node {
	struct ubifs_ch ch;
	__le16 child_cnt;
	__le16 level;
	__u8 branches[];
} __attribute__ ((packed));

struct ubifs_cs_node {
	struct ubifs_ch ch;
	__le64 cmt_no;
} __attribute__ ((packed));

struct ubifs_orph_node {
	struct ubifs_ch ch;
	__le64 cmt_no;
	__le64 inos[];
} __attribute__ ((packed));

#endif /* __UBIFS_MEDIA_H__ */
