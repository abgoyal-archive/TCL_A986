

#ifndef _LINUX_SUNRPC_XPRT_RDMA_H
#define _LINUX_SUNRPC_XPRT_RDMA_H

#include <linux/wait.h> 		/* wait_queue_head_t, etc */
#include <linux/spinlock.h> 		/* spinlock_t, etc */
#include <asm/atomic.h>			/* atomic_t, etc */

#include <rdma/rdma_cm.h>		/* RDMA connection api */
#include <rdma/ib_verbs.h>		/* RDMA verbs api */

#include <linux/sunrpc/clnt.h> 		/* rpc_xprt */
#include <linux/sunrpc/rpc_rdma.h> 	/* RPC/RDMA protocol */
#include <linux/sunrpc/xprtrdma.h> 	/* xprt parameters */

#define RDMA_RESOLVE_TIMEOUT	(5000)	/* 5 seconds */
#define RDMA_CONNECT_RETRY_MAX	(2)	/* retries if no listener backlog */

struct rpcrdma_ia {
	struct rdma_cm_id 	*ri_id;
	struct ib_pd		*ri_pd;
	struct ib_mr		*ri_bind_mem;
	u32			ri_dma_lkey;
	int			ri_have_dma_lkey;
	struct completion	ri_done;
	int			ri_async_rc;
	enum rpcrdma_memreg	ri_memreg_strategy;
};


struct rpcrdma_ep {
	atomic_t		rep_cqcount;
	int			rep_cqinit;
	int			rep_connected;
	struct rpcrdma_ia	*rep_ia;
	struct ib_cq		*rep_cq;
	struct ib_qp_init_attr	rep_attr;
	wait_queue_head_t 	rep_connect_wait;
	struct ib_sge		rep_pad;	/* holds zeroed pad */
	struct ib_mr		*rep_pad_mr;	/* holds zeroed pad */
	void			(*rep_func)(struct rpcrdma_ep *);
	struct rpc_xprt		*rep_xprt;	/* for rep_func */
	struct rdma_conn_param	rep_remote_cma;
	struct sockaddr_storage	rep_remote_addr;
};

#define INIT_CQCOUNT(ep) atomic_set(&(ep)->rep_cqcount, (ep)->rep_cqinit)
#define DECR_CQCOUNT(ep) atomic_sub_return(1, &(ep)->rep_cqcount)


/* temporary static scatter/gather max */
#define RPCRDMA_MAX_DATA_SEGS	(8)	/* max scatter/gather */
#define RPCRDMA_MAX_SEGS 	(RPCRDMA_MAX_DATA_SEGS + 2) /* head+tail = 2 */
#define MAX_RPCRDMAHDR	(\
	/* max supported RPC/RDMA header */ \
	sizeof(struct rpcrdma_msg) + (2 * sizeof(u32)) + \
	(sizeof(struct rpcrdma_read_chunk) * RPCRDMA_MAX_SEGS) + sizeof(u32))

struct rpcrdma_buffer;

struct rpcrdma_rep {
	unsigned int	rr_len;		/* actual received reply length */
	struct rpcrdma_buffer *rr_buffer; /* home base for this structure */
	struct rpc_xprt	*rr_xprt;	/* needed for request/reply matching */
	void (*rr_func)(struct rpcrdma_rep *);/* called by tasklet in softint */
	struct list_head rr_list;	/* tasklet list */
	wait_queue_head_t rr_unbind;	/* optional unbind wait */
	struct ib_sge	rr_iov;		/* for posting */
	struct ib_mr	*rr_handle;	/* handle for mem in rr_iov */
	char	rr_base[MAX_RPCRDMAHDR]; /* minimal inline receive buffer */
};


struct rpcrdma_mr_seg {		/* chunk descriptors */
	union {				/* chunk memory handles */
		struct ib_mr	*rl_mr;		/* if registered directly */
		struct rpcrdma_mw {		/* if registered from region */
			union {
				struct ib_mw	*mw;
				struct ib_fmr	*fmr;
				struct {
					struct ib_fast_reg_page_list *fr_pgl;
					struct ib_mr *fr_mr;
				} frmr;
			} r;
			struct list_head mw_list;
		} *rl_mw;
	} mr_chunk;
	u64		mr_base;	/* registration result */
	u32		mr_rkey;	/* registration result */
	u32		mr_len;		/* length of chunk or segment */
	int		mr_nsegs;	/* number of segments in chunk or 0 */
	enum dma_data_direction	mr_dir;	/* segment mapping direction */
	dma_addr_t	mr_dma;		/* segment mapping address */
	size_t		mr_dmalen;	/* segment mapping length */
	struct page	*mr_page;	/* owning page, if any */
	char		*mr_offset;	/* kva if no page, else offset */
};

struct rpcrdma_req {
	size_t 		rl_size;	/* actual length of buffer */
	unsigned int	rl_niovs;	/* 0, 2 or 4 */
	unsigned int	rl_nchunks;	/* non-zero if chunks */
	unsigned int	rl_connect_cookie;	/* retry detection */
	struct rpcrdma_buffer *rl_buffer; /* home base for this structure */
	struct rpcrdma_rep	*rl_reply;/* holder for reply buffer */
	struct rpcrdma_mr_seg rl_segments[RPCRDMA_MAX_SEGS];/* chunk segments */
	struct ib_sge	rl_send_iov[4];	/* for active requests */
	struct ib_sge	rl_iov;		/* for posting */
	struct ib_mr	*rl_handle;	/* handle for mem in rl_iov */
	char		rl_base[MAX_RPCRDMAHDR]; /* start of actual buffer */
	__u32 		rl_xdr_buf[0];	/* start of returned rpc rq_buffer */
};
#define rpcr_to_rdmar(r) \
	container_of((r)->rq_buffer, struct rpcrdma_req, rl_xdr_buf[0])

struct rpcrdma_buffer {
	spinlock_t	rb_lock;	/* protects indexes */
	atomic_t	rb_credits;	/* most recent server credits */
	unsigned long	rb_cwndscale;	/* cached framework rpc_cwndscale */
	int		rb_max_requests;/* client max requests */
	struct list_head rb_mws;	/* optional memory windows/fmrs/frmrs */
	int		rb_send_index;
	struct rpcrdma_req	**rb_send_bufs;
	int		rb_recv_index;
	struct rpcrdma_rep	**rb_recv_bufs;
	char		*rb_pool;
};
#define rdmab_to_ia(b) (&container_of((b), struct rpcrdma_xprt, rx_buf)->rx_ia)

struct rpcrdma_create_data_internal {
	struct sockaddr_storage	addr;	/* RDMA server address */
	unsigned int	max_requests;	/* max requests (slots) in flight */
	unsigned int	rsize;		/* mount rsize - max read hdr+data */
	unsigned int	wsize;		/* mount wsize - max write hdr+data */
	unsigned int	inline_rsize;	/* max non-rdma read data payload */
	unsigned int	inline_wsize;	/* max non-rdma write data payload */
	unsigned int	padding;	/* non-rdma write header padding */
};

#define RPCRDMA_INLINE_READ_THRESHOLD(rq) \
	(rpcx_to_rdmad(rq->rq_task->tk_xprt).inline_rsize)

#define RPCRDMA_INLINE_WRITE_THRESHOLD(rq)\
	(rpcx_to_rdmad(rq->rq_task->tk_xprt).inline_wsize)

#define RPCRDMA_INLINE_PAD_VALUE(rq)\
	rpcx_to_rdmad(rq->rq_task->tk_xprt).padding

struct rpcrdma_stats {
	unsigned long		read_chunk_count;
	unsigned long		write_chunk_count;
	unsigned long		reply_chunk_count;

	unsigned long long	total_rdma_request;
	unsigned long long	total_rdma_reply;

	unsigned long long	pullup_copy_count;
	unsigned long long	fixup_copy_count;
	unsigned long		hardway_register_count;
	unsigned long		failed_marshal_count;
	unsigned long		bad_reply_count;
};

struct rpcrdma_xprt {
	struct rpc_xprt		xprt;
	struct rpcrdma_ia	rx_ia;
	struct rpcrdma_ep	rx_ep;
	struct rpcrdma_buffer	rx_buf;
	struct rpcrdma_create_data_internal rx_data;
	struct delayed_work	rdma_connect;
	struct rpcrdma_stats	rx_stats;
};

#define rpcx_to_rdmax(x) container_of(x, struct rpcrdma_xprt, xprt)
#define rpcx_to_rdmad(x) (rpcx_to_rdmax(x)->rx_data)

extern int xprt_rdma_pad_optimize;

int rpcrdma_ia_open(struct rpcrdma_xprt *, struct sockaddr *, int);
void rpcrdma_ia_close(struct rpcrdma_ia *);

int rpcrdma_ep_create(struct rpcrdma_ep *, struct rpcrdma_ia *,
				struct rpcrdma_create_data_internal *);
int rpcrdma_ep_destroy(struct rpcrdma_ep *, struct rpcrdma_ia *);
int rpcrdma_ep_connect(struct rpcrdma_ep *, struct rpcrdma_ia *);
int rpcrdma_ep_disconnect(struct rpcrdma_ep *, struct rpcrdma_ia *);

int rpcrdma_ep_post(struct rpcrdma_ia *, struct rpcrdma_ep *,
				struct rpcrdma_req *);
int rpcrdma_ep_post_recv(struct rpcrdma_ia *, struct rpcrdma_ep *,
				struct rpcrdma_rep *);

int rpcrdma_buffer_create(struct rpcrdma_buffer *, struct rpcrdma_ep *,
				struct rpcrdma_ia *,
				struct rpcrdma_create_data_internal *);
void rpcrdma_buffer_destroy(struct rpcrdma_buffer *);

struct rpcrdma_req *rpcrdma_buffer_get(struct rpcrdma_buffer *);
void rpcrdma_buffer_put(struct rpcrdma_req *);
void rpcrdma_recv_buffer_get(struct rpcrdma_req *);
void rpcrdma_recv_buffer_put(struct rpcrdma_rep *);

int rpcrdma_register_internal(struct rpcrdma_ia *, void *, int,
				struct ib_mr **, struct ib_sge *);
int rpcrdma_deregister_internal(struct rpcrdma_ia *,
				struct ib_mr *, struct ib_sge *);

int rpcrdma_register_external(struct rpcrdma_mr_seg *,
				int, int, struct rpcrdma_xprt *);
int rpcrdma_deregister_external(struct rpcrdma_mr_seg *,
				struct rpcrdma_xprt *, void *);

void rpcrdma_conn_func(struct rpcrdma_ep *);
void rpcrdma_reply_handler(struct rpcrdma_rep *);

int rpcrdma_marshal_req(struct rpc_rqst *);

#endif				/* _LINUX_SUNRPC_XPRT_RDMA_H */
