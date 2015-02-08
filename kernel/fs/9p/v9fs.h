
#include <linux/backing-dev.h>

enum p9_session_flags {
	V9FS_PROTO_2000U	= 0x01,
	V9FS_PROTO_2000L	= 0x02,
	V9FS_ACCESS_SINGLE	= 0x04,
	V9FS_ACCESS_USER	= 0x08,
	V9FS_ACCESS_ANY		= 0x0C,
	V9FS_ACCESS_MASK	= 0x0C,
};

/* possible values of ->cache */

enum p9_cache_modes {
	CACHE_NONE,
	CACHE_LOOSE,
	CACHE_FSCACHE,
};


struct v9fs_session_info {
	/* options */
	unsigned char flags;
	unsigned char nodev;
	unsigned short debug;
	unsigned int afid;
	unsigned int cache;
#ifdef CONFIG_9P_FSCACHE
	char *cachetag;
	struct fscache_cookie *fscache;
#endif

	char *uname;		/* user name to mount as */
	char *aname;		/* name of remote hierarchy being mounted */
	unsigned int maxdata;	/* max data for client interface */
	unsigned int dfltuid;	/* default uid/muid for legacy support */
	unsigned int dfltgid;	/* default gid for legacy support */
	u32 uid;		/* if ACCESS_SINGLE, the uid that has access */
	struct p9_client *clnt;	/* 9p client */
	struct list_head slist; /* list of sessions registered with v9fs */
	struct backing_dev_info bdi;
};

struct p9_fid *v9fs_session_init(struct v9fs_session_info *, const char *,
									char *);
void v9fs_session_close(struct v9fs_session_info *v9ses);
void v9fs_session_cancel(struct v9fs_session_info *v9ses);
void v9fs_session_begin_cancel(struct v9fs_session_info *v9ses);

#define V9FS_MAGIC 0x01021997

/* other default globals */
#define V9FS_PORT	564
#define V9FS_DEFUSER	"nobody"
#define V9FS_DEFANAME	""
#define V9FS_DEFUID	(-2)
#define V9FS_DEFGID	(-2)

static inline struct v9fs_session_info *v9fs_inode2v9ses(struct inode *inode)
{
	return (inode->i_sb->s_fs_info);
}

static inline int v9fs_proto_dotu(struct v9fs_session_info *v9ses)
{
	return v9ses->flags & V9FS_PROTO_2000U;
}

static inline int v9fs_proto_dotl(struct v9fs_session_info *v9ses)
{
	return v9ses->flags & V9FS_PROTO_2000L;
}
