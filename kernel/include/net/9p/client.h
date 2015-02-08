

#ifndef NET_9P_CLIENT_H
#define NET_9P_CLIENT_H

/* Number of requests per row */
#define P9_ROW_MAXTAG 255


enum p9_proto_versions{
	p9_proto_legacy = 0,
	p9_proto_2000u = 1,
	p9_proto_2000L = 2,
};



enum p9_trans_status {
	Connected,
	BeginDisconnect,
	Disconnected,
	Hung,
};


enum p9_req_status_t {
	REQ_STATUS_IDLE,
	REQ_STATUS_ALLOC,
	REQ_STATUS_UNSENT,
	REQ_STATUS_SENT,
	REQ_STATUS_FLSH,
	REQ_STATUS_RCVD,
	REQ_STATUS_FLSHD,
	REQ_STATUS_ERROR,
};


struct p9_req_t {
	int status;
	int t_err;
	wait_queue_head_t *wq;
	struct p9_fcall *tc;
	struct p9_fcall *rc;
	void *aux;

	struct list_head req_list;
};


struct p9_client {
	spinlock_t lock; /* protect client structure */
	int msize;
	unsigned char proto_version;
	struct p9_trans_module *trans_mod;
	enum p9_trans_status status;
	void *trans;
	struct p9_conn *conn;

	struct p9_idpool *fidpool;
	struct list_head fidlist;

	struct p9_idpool *tagpool;
	struct p9_req_t *reqs[P9_ROW_MAXTAG];
	int max_tag;
};


struct p9_fid {
	struct p9_client *clnt;
	u32 fid;
	int mode;
	struct p9_qid qid;
	u32 iounit;
	uid_t uid;

	void *rdir;

	struct list_head flist;
	struct list_head dlist;	/* list of all fids attached to a dentry */
};

int p9_client_statfs(struct p9_fid *fid, struct p9_rstatfs *sb);
int p9_client_rename(struct p9_fid *fid, struct p9_fid *newdirfid, char *name);
int p9_client_version(struct p9_client *);
struct p9_client *p9_client_create(const char *dev_name, char *options);
void p9_client_destroy(struct p9_client *clnt);
void p9_client_disconnect(struct p9_client *clnt);
void p9_client_begin_disconnect(struct p9_client *clnt);
struct p9_fid *p9_client_attach(struct p9_client *clnt, struct p9_fid *afid,
					char *uname, u32 n_uname, char *aname);
struct p9_fid *p9_client_auth(struct p9_client *clnt, char *uname,
						u32 n_uname, char *aname);
struct p9_fid *p9_client_walk(struct p9_fid *oldfid, int nwname, char **wnames,
								int clone);
int p9_client_open(struct p9_fid *fid, int mode);
int p9_client_fcreate(struct p9_fid *fid, char *name, u32 perm, int mode,
							char *extension);
int p9_client_clunk(struct p9_fid *fid);
int p9_client_remove(struct p9_fid *fid);
int p9_client_read(struct p9_fid *fid, char *data, char __user *udata,
							u64 offset, u32 count);
int p9_client_write(struct p9_fid *fid, char *data, const char __user *udata,
							u64 offset, u32 count);
struct p9_wstat *p9_client_stat(struct p9_fid *fid);
int p9_client_wstat(struct p9_fid *fid, struct p9_wstat *wst);

struct p9_req_t *p9_tag_lookup(struct p9_client *, u16);
void p9_client_cb(struct p9_client *c, struct p9_req_t *req);

int p9_parse_header(struct p9_fcall *, int32_t *, int8_t *, int16_t *, int);
int p9stat_read(char *, int, struct p9_wstat *, int);
void p9stat_free(struct p9_wstat *);

int p9_is_proto_dotu(struct p9_client *clnt);
int p9_is_proto_dotl(struct p9_client *clnt);

#endif /* NET_9P_CLIENT_H */
