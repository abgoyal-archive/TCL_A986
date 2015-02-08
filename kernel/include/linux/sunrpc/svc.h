


#ifndef SUNRPC_SVC_H
#define SUNRPC_SVC_H

#include <linux/in.h>
#include <linux/in6.h>
#include <linux/sunrpc/types.h>
#include <linux/sunrpc/xdr.h>
#include <linux/sunrpc/auth.h>
#include <linux/sunrpc/svcauth.h>
#include <linux/wait.h>
#include <linux/mm.h>

typedef int		(*svc_thread_fn)(void *);

/* statistics for svc_pool structures */
struct svc_pool_stats {
	unsigned long	packets;
	unsigned long	sockets_queued;
	unsigned long	threads_woken;
	unsigned long	threads_timedout;
};

struct svc_pool {
	unsigned int		sp_id;	    	/* pool id; also node id on NUMA */
	spinlock_t		sp_lock;	/* protects all fields */
	struct list_head	sp_threads;	/* idle server threads */
	struct list_head	sp_sockets;	/* pending sockets */
	unsigned int		sp_nrthreads;	/* # of threads in pool */
	struct list_head	sp_all_threads;	/* all server threads */
	struct svc_pool_stats	sp_stats;	/* statistics on pool operation */
} ____cacheline_aligned_in_smp;

struct svc_serv {
	struct svc_program *	sv_program;	/* RPC program */
	struct svc_stat *	sv_stats;	/* RPC statistics */
	spinlock_t		sv_lock;
	unsigned int		sv_nrthreads;	/* # of server threads */
	unsigned int		sv_maxconn;	/* max connections allowed or
						 * '0' causing max to be based
						 * on number of threads. */

	unsigned int		sv_max_payload;	/* datagram payload size */
	unsigned int		sv_max_mesg;	/* max_payload + 1 page for overheads */
	unsigned int		sv_xdrsize;	/* XDR buffer size */
	struct list_head	sv_permsocks;	/* all permanent sockets */
	struct list_head	sv_tempsocks;	/* all temporary sockets */
	int			sv_tmpcnt;	/* count of temporary sockets */
	struct timer_list	sv_temptimer;	/* timer for aging temporary sockets */

	char *			sv_name;	/* service name */

	unsigned int		sv_nrpools;	/* number of thread pools */
	struct svc_pool *	sv_pools;	/* array of thread pools */

	void			(*sv_shutdown)(struct svc_serv *serv);
						/* Callback to use when last thread
						 * exits.
						 */

	struct module *		sv_module;	/* optional module to count when
						 * adding threads */
	svc_thread_fn		sv_function;	/* main function for threads */
#if defined(CONFIG_NFS_V4_1)
	struct list_head	sv_cb_list;	/* queue for callback requests
						 * that arrive over the same
						 * connection */
	spinlock_t		sv_cb_lock;	/* protects the svc_cb_list */
	wait_queue_head_t	sv_cb_waitq;	/* sleep here if there are no
						 * entries in the svc_cb_list */
	struct svc_xprt		*bc_xprt;
#endif /* CONFIG_NFS_V4_1 */
};

static inline void svc_get(struct svc_serv *serv)
{
	serv->sv_nrthreads++;
}

#define RPCSVC_MAXPAYLOAD	(1*1024*1024u)
#define RPCSVC_MAXPAYLOAD_TCP	RPCSVC_MAXPAYLOAD
#define RPCSVC_MAXPAYLOAD_UDP	(32*1024u)

extern u32 svc_max_payload(const struct svc_rqst *rqstp);

#define RPCSVC_MAXPAGES		((RPCSVC_MAXPAYLOAD+PAGE_SIZE-1)/PAGE_SIZE \
				+ 2 + 1)

static inline u32 svc_getnl(struct kvec *iov)
{
	__be32 val, *vp;
	vp = iov->iov_base;
	val = *vp++;
	iov->iov_base = (void*)vp;
	iov->iov_len -= sizeof(__be32);
	return ntohl(val);
}

static inline void svc_putnl(struct kvec *iov, u32 val)
{
	__be32 *vp = iov->iov_base + iov->iov_len;
	*vp = htonl(val);
	iov->iov_len += sizeof(__be32);
}

static inline __be32 svc_getu32(struct kvec *iov)
{
	__be32 val, *vp;
	vp = iov->iov_base;
	val = *vp++;
	iov->iov_base = (void*)vp;
	iov->iov_len -= sizeof(__be32);
	return val;
}

static inline void svc_ungetu32(struct kvec *iov)
{
	__be32 *vp = (__be32 *)iov->iov_base;
	iov->iov_base = (void *)(vp - 1);
	iov->iov_len += sizeof(*vp);
}

static inline void svc_putu32(struct kvec *iov, __be32 val)
{
	__be32 *vp = iov->iov_base + iov->iov_len;
	*vp = val;
	iov->iov_len += sizeof(__be32);
}

union svc_addr_u {
    struct in_addr	addr;
    struct in6_addr	addr6;
};

struct svc_rqst {
	struct list_head	rq_list;	/* idle list */
	struct list_head	rq_all;		/* all threads list */
	struct svc_xprt *	rq_xprt;	/* transport ptr */
	struct sockaddr_storage	rq_addr;	/* peer address */
	size_t			rq_addrlen;

	struct svc_serv *	rq_server;	/* RPC service definition */
	struct svc_pool *	rq_pool;	/* thread pool */
	struct svc_procedure *	rq_procinfo;	/* procedure info */
	struct auth_ops *	rq_authop;	/* authentication flavour */
	u32			rq_flavor;	/* pseudoflavor */
	struct svc_cred		rq_cred;	/* auth info */
	void *			rq_xprt_ctxt;	/* transport specific context ptr */
	struct svc_deferred_req*rq_deferred;	/* deferred request we are replaying */
	int			rq_usedeferral;	/* use deferral */

	size_t			rq_xprt_hlen;	/* xprt header len */
	struct xdr_buf		rq_arg;
	struct xdr_buf		rq_res;
	struct page *		rq_pages[RPCSVC_MAXPAGES];
	struct page *		*rq_respages;	/* points into rq_pages */
	int			rq_resused;	/* number of pages used for result */

	struct kvec		rq_vec[RPCSVC_MAXPAGES]; /* generally useful.. */

	__be32			rq_xid;		/* transmission id */
	u32			rq_prog;	/* program number */
	u32			rq_vers;	/* program version */
	u32			rq_proc;	/* procedure number */
	u32			rq_prot;	/* IP protocol */
	unsigned short
				rq_secure  : 1;	/* secure port */

	union svc_addr_u	rq_daddr;	/* dest addr of request
						 *  - reply from here */

	void *			rq_argp;	/* decoded arguments */
	void *			rq_resp;	/* xdr'd results */
	void *			rq_auth_data;	/* flavor-specific data */

	int			rq_reserved;	/* space on socket outq
						 * reserved for this request
						 */

	struct cache_req	rq_chandle;	/* handle passed to caches for 
						 * request delaying 
						 */
	/* Catering to nfsd */
	struct auth_domain *	rq_client;	/* RPC peer info */
	struct auth_domain *	rq_gssclient;	/* "gss/"-style peer info */
	struct svc_cacherep *	rq_cacherep;	/* cache info */
	int			rq_splice_ok;   /* turned off in gss privacy
						 * to prevent encrypting page
						 * cache pages */
	wait_queue_head_t	rq_wait;	/* synchronization */
	struct task_struct	*rq_task;	/* service thread */
};

static inline struct sockaddr_in *svc_addr_in(const struct svc_rqst *rqst)
{
	return (struct sockaddr_in *) &rqst->rq_addr;
}

static inline struct sockaddr_in6 *svc_addr_in6(const struct svc_rqst *rqst)
{
	return (struct sockaddr_in6 *) &rqst->rq_addr;
}

static inline struct sockaddr *svc_addr(const struct svc_rqst *rqst)
{
	return (struct sockaddr *) &rqst->rq_addr;
}

static inline int
xdr_argsize_check(struct svc_rqst *rqstp, __be32 *p)
{
	char *cp = (char *)p;
	struct kvec *vec = &rqstp->rq_arg.head[0];
	return cp >= (char*)vec->iov_base
		&& cp <= (char*)vec->iov_base + vec->iov_len;
}

static inline int
xdr_ressize_check(struct svc_rqst *rqstp, __be32 *p)
{
	struct kvec *vec = &rqstp->rq_res.head[0];
	char *cp = (char*)p;

	vec->iov_len = cp - (char*)vec->iov_base;

	return vec->iov_len <= PAGE_SIZE;
}

static inline void svc_free_res_pages(struct svc_rqst *rqstp)
{
	while (rqstp->rq_resused) {
		struct page **pp = (rqstp->rq_respages +
				    --rqstp->rq_resused);
		if (*pp) {
			put_page(*pp);
			*pp = NULL;
		}
	}
}

struct svc_deferred_req {
	u32			prot;	/* protocol (UDP or TCP) */
	struct svc_xprt		*xprt;
	struct sockaddr_storage	addr;	/* where reply must go */
	size_t			addrlen;
	union svc_addr_u	daddr;	/* where reply must come from */
	struct cache_deferred_req handle;
	size_t			xprt_hlen;
	int			argslen;
	__be32			args[0];
};

struct svc_program {
	struct svc_program *	pg_next;	/* other programs (same xprt) */
	u32			pg_prog;	/* program number */
	unsigned int		pg_lovers;	/* lowest version */
	unsigned int		pg_hivers;	/* lowest version */
	unsigned int		pg_nvers;	/* number of versions */
	struct svc_version **	pg_vers;	/* version array */
	char *			pg_name;	/* service name */
	char *			pg_class;	/* class name: services sharing authentication */
	struct svc_stat *	pg_stats;	/* rpc statistics */
	int			(*pg_authenticate)(struct svc_rqst *);
};

struct svc_version {
	u32			vs_vers;	/* version number */
	u32			vs_nproc;	/* number of procedures */
	struct svc_procedure *	vs_proc;	/* per-procedure info */
	u32			vs_xdrsize;	/* xdrsize needed for this version */

	unsigned int		vs_hidden : 1;	/* Don't register with portmapper.
						 * Only used for nfsacl so far. */

	/* Override dispatch function (e.g. when caching replies).
	 * A return value of 0 means drop the request. 
	 * vs_dispatch == NULL means use default dispatcher.
	 */
	int			(*vs_dispatch)(struct svc_rqst *, __be32 *);
};

typedef __be32	(*svc_procfunc)(struct svc_rqst *, void *argp, void *resp);
struct svc_procedure {
	svc_procfunc		pc_func;	/* process the request */
	kxdrproc_t		pc_decode;	/* XDR decode args */
	kxdrproc_t		pc_encode;	/* XDR encode result */
	kxdrproc_t		pc_release;	/* XDR free result */
	unsigned int		pc_argsize;	/* argument struct size */
	unsigned int		pc_ressize;	/* result struct size */
	unsigned int		pc_count;	/* call count */
	unsigned int		pc_cachetype;	/* cache info (NFS) */
	unsigned int		pc_xdrressize;	/* maximum size of XDR reply */
};

struct svc_serv *svc_create(struct svc_program *, unsigned int,
			    void (*shutdown)(struct svc_serv *));
struct svc_rqst *svc_prepare_thread(struct svc_serv *serv,
					struct svc_pool *pool);
void		   svc_exit_thread(struct svc_rqst *);
struct svc_serv *  svc_create_pooled(struct svc_program *, unsigned int,
			void (*shutdown)(struct svc_serv *),
			svc_thread_fn, struct module *);
int		   svc_set_num_threads(struct svc_serv *, struct svc_pool *, int);
int		   svc_pool_stats_open(struct svc_serv *serv, struct file *file);
void		   svc_destroy(struct svc_serv *);
int		   svc_process(struct svc_rqst *);
int		   bc_svc_process(struct svc_serv *, struct rpc_rqst *,
			struct svc_rqst *);
int		   svc_register(const struct svc_serv *, const int,
				const unsigned short, const unsigned short);

void		   svc_wake_up(struct svc_serv *);
void		   svc_reserve(struct svc_rqst *rqstp, int space);
struct svc_pool *  svc_pool_for_cpu(struct svc_serv *serv, int cpu);
char *		   svc_print_addr(struct svc_rqst *, char *, size_t);

#define	RPC_MAX_ADDRBUFLEN	(63U)

static inline void svc_reserve_auth(struct svc_rqst *rqstp, int space)
{
	int added_space = 0;

	if (rqstp->rq_authop->flavour)
		added_space = RPC_MAX_AUTH_SIZE;
	svc_reserve(rqstp, space + added_space);
}

#endif /* SUNRPC_SVC_H */
