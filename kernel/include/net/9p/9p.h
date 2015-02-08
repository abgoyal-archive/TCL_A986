

#ifndef NET_9P_H
#define NET_9P_H


enum p9_debug_flags {
	P9_DEBUG_ERROR = 	(1<<0),
	P9_DEBUG_9P = 		(1<<2),
	P9_DEBUG_VFS =		(1<<3),
	P9_DEBUG_CONV =		(1<<4),
	P9_DEBUG_MUX =		(1<<5),
	P9_DEBUG_TRANS =	(1<<6),
	P9_DEBUG_SLABS =      	(1<<7),
	P9_DEBUG_FCALL =	(1<<8),
	P9_DEBUG_FID =		(1<<9),
	P9_DEBUG_PKT =		(1<<10),
	P9_DEBUG_FSC =		(1<<11),
};

#ifdef CONFIG_NET_9P_DEBUG
extern unsigned int p9_debug_level;

#define P9_DPRINTK(level, format, arg...) \
do {  \
	if ((p9_debug_level & level) == level) {\
		if (level == P9_DEBUG_9P) \
			printk(KERN_NOTICE "(%8.8d) " \
			format , task_pid_nr(current) , ## arg); \
		else \
			printk(KERN_NOTICE "-- %s (%d): " \
			format , __func__, task_pid_nr(current) , ## arg); \
	} \
} while (0)

#else
#define P9_DPRINTK(level, format, arg...)  do { } while (0)
#endif

#define P9_EPRINTK(level, format, arg...) \
do { \
	printk(level "9p: %s (%d): " \
		format , __func__, task_pid_nr(current), ## arg); \
} while (0)


enum p9_msg_t {
	P9_TSTATFS = 8,
	P9_RSTATFS,
	P9_TRENAME = 20,
	P9_RRENAME,
	P9_TVERSION = 100,
	P9_RVERSION,
	P9_TAUTH = 102,
	P9_RAUTH,
	P9_TATTACH = 104,
	P9_RATTACH,
	P9_TERROR = 106,
	P9_RERROR,
	P9_TFLUSH = 108,
	P9_RFLUSH,
	P9_TWALK = 110,
	P9_RWALK,
	P9_TOPEN = 112,
	P9_ROPEN,
	P9_TCREATE = 114,
	P9_RCREATE,
	P9_TREAD = 116,
	P9_RREAD,
	P9_TWRITE = 118,
	P9_RWRITE,
	P9_TCLUNK = 120,
	P9_RCLUNK,
	P9_TREMOVE = 122,
	P9_RREMOVE,
	P9_TSTAT = 124,
	P9_RSTAT,
	P9_TWSTAT = 126,
	P9_RWSTAT,
};


enum p9_open_mode_t {
	P9_OREAD = 0x00,
	P9_OWRITE = 0x01,
	P9_ORDWR = 0x02,
	P9_OEXEC = 0x03,
	P9_OTRUNC = 0x10,
	P9_OREXEC = 0x20,
	P9_ORCLOSE = 0x40,
	P9_OAPPEND = 0x80,
	P9_OEXCL = 0x1000,
};

enum p9_perm_t {
	P9_DMDIR = 0x80000000,
	P9_DMAPPEND = 0x40000000,
	P9_DMEXCL = 0x20000000,
	P9_DMMOUNT = 0x10000000,
	P9_DMAUTH = 0x08000000,
	P9_DMTMP = 0x04000000,
/* 9P2000.u extensions */
	P9_DMSYMLINK = 0x02000000,
	P9_DMLINK = 0x01000000,
	P9_DMDEVICE = 0x00800000,
	P9_DMNAMEDPIPE = 0x00200000,
	P9_DMSOCKET = 0x00100000,
	P9_DMSETUID = 0x00080000,
	P9_DMSETGID = 0x00040000,
	P9_DMSETVTX = 0x00010000,
};

enum p9_qid_t {
	P9_QTDIR = 0x80,
	P9_QTAPPEND = 0x40,
	P9_QTEXCL = 0x20,
	P9_QTMOUNT = 0x10,
	P9_QTAUTH = 0x08,
	P9_QTTMP = 0x04,
	P9_QTSYMLINK = 0x02,
	P9_QTLINK = 0x01,
	P9_QTFILE = 0x00,
};

/* 9P Magic Numbers */
#define P9_NOTAG	(u16)(~0)
#define P9_NOFID	(u32)(~0)
#define P9_MAXWELEM	16

/* ample room for Twrite/Rread header */
#define P9_IOHDRSZ	24


struct p9_str {
	u16 len;
	char *str;
};


struct p9_qid {
	u8 type;
	u32 version;
	u64 path;
};


struct p9_wstat {
	u16 size;
	u16 type;
	u32 dev;
	struct p9_qid qid;
	u32 mode;
	u32 atime;
	u32 mtime;
	u64 length;
	char *name;
	char *uid;
	char *gid;
	char *muid;
	char *extension;	/* 9p2000.u extensions */
	u32 n_uid;		/* 9p2000.u extensions */
	u32 n_gid;		/* 9p2000.u extensions */
	u32 n_muid;		/* 9p2000.u extensions */
};

/* Structures for Protocol Operations */
struct p9_tstatfs {
	u32 fid;
};

struct p9_rstatfs {
	u32 type;
	u32 bsize;
	u64 blocks;
	u64 bfree;
	u64 bavail;
	u64 files;
	u64 ffree;
	u64 fsid;
	u32 namelen;
};

struct p9_trename {
	u32 fid;
	u32 newdirfid;
	struct p9_str name;
};

struct p9_rrename {
};

struct p9_tversion {
	u32 msize;
	struct p9_str version;
};

struct p9_rversion {
	u32 msize;
	struct p9_str version;
};

struct p9_tauth {
	u32 afid;
	struct p9_str uname;
	struct p9_str aname;
	u32 n_uname;		/* 9P2000.u extensions */
};

struct p9_rauth {
	struct p9_qid qid;
};

struct p9_rerror {
	struct p9_str error;
	u32 errno;		/* 9p2000.u extension */
};

struct p9_tflush {
	u16 oldtag;
};

struct p9_rflush {
};

struct p9_tattach {
	u32 fid;
	u32 afid;
	struct p9_str uname;
	struct p9_str aname;
	u32 n_uname;		/* 9P2000.u extensions */
};

struct p9_rattach {
	struct p9_qid qid;
};

struct p9_twalk {
	u32 fid;
	u32 newfid;
	u16 nwname;
	struct p9_str wnames[16];
};

struct p9_rwalk {
	u16 nwqid;
	struct p9_qid wqids[16];
};

struct p9_topen {
	u32 fid;
	u8 mode;
};

struct p9_ropen {
	struct p9_qid qid;
	u32 iounit;
};

struct p9_tcreate {
	u32 fid;
	struct p9_str name;
	u32 perm;
	u8 mode;
	struct p9_str extension;
};

struct p9_rcreate {
	struct p9_qid qid;
	u32 iounit;
};

struct p9_tread {
	u32 fid;
	u64 offset;
	u32 count;
};

struct p9_rread {
	u32 count;
	u8 *data;
};

struct p9_twrite {
	u32 fid;
	u64 offset;
	u32 count;
	u8 *data;
};

struct p9_rwrite {
	u32 count;
};

struct p9_tclunk {
	u32 fid;
};

struct p9_rclunk {
};

struct p9_tremove {
	u32 fid;
};

struct p9_rremove {
};

struct p9_tstat {
	u32 fid;
};

struct p9_rstat {
	struct p9_wstat stat;
};

struct p9_twstat {
	u32 fid;
	struct p9_wstat stat;
};

struct p9_rwstat {
};


struct p9_fcall {
	u32 size;
	u8 id;
	u16 tag;

	size_t offset;
	size_t capacity;

	uint8_t *sdata;
};

struct p9_idpool;

int p9_errstr2errno(char *errstr, int len);

struct p9_idpool *p9_idpool_create(void);
void p9_idpool_destroy(struct p9_idpool *);
int p9_idpool_get(struct p9_idpool *p);
void p9_idpool_put(int id, struct p9_idpool *p);
int p9_idpool_check(int id, struct p9_idpool *p);

int p9_error_init(void);
int p9_errstr2errno(char *, int);
int p9_trans_fd_init(void);
void p9_trans_fd_exit(void);
#endif /* NET_9P_H */
