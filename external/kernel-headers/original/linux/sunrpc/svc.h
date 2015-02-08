

#ifndef SUNRPC_SVC_H
#define SUNRPC_SVC_H

#include <linux/in.h>
#include <linux/sunrpc/types.h>
#include <linux/sunrpc/xdr.h>
#include <linux/sunrpc/svcauth.h>
#include <linux/wait.h>
#include <linux/mm.h>

struct svc_serv {
	struct list_head	sv_threads;	/* idle server threads */
	struct list_head	sv_sockets;	/* pending sockets */
	struct svc_program *	sv_program;	/* RPC program */
	struct svc_stat *	sv_stats;	/* RPC statistics */
	spinlock_t		sv_lock;
	unsigned int		sv_nrthreads;	/* # of server threads */
	unsigned int		sv_bufsz;	/* datagram buffer size */
	unsigned int		sv_xdrsize;	/* XDR buffer size */

	struct list_head	sv_permsocks;	/* all permanent sockets */
	struct list_head	sv_tempsocks;	/* all temporary sockets */
	int			sv_tmpcnt;	/* count of temporary sockets */

	char *			sv_name;	/* service name */
};

#define RPCSVC_MAXPAYLOAD	(64*1024u)

#define RPCSVC_MAXPAGES		((RPCSVC_MAXPAYLOAD+PAGE_SIZE-1)/PAGE_SIZE + 2)

static inline u32 svc_getu32(struct kvec *iov)
{
	u32 val, *vp;
	vp = iov->iov_base;
	val = *vp++;
	iov->iov_base = (void*)vp;
	iov->iov_len -= sizeof(u32);
	return val;
}

static inline void svc_ungetu32(struct kvec *iov)
{
	u32 *vp = (u32 *)iov->iov_base;
	iov->iov_base = (void *)(vp - 1);
	iov->iov_len += sizeof(*vp);
}

static inline void svc_putu32(struct kvec *iov, u32 val)
{
	u32 *vp = iov->iov_base + iov->iov_len;
	*vp = val;
	iov->iov_len += sizeof(u32);
}

	
struct svc_rqst {
	struct list_head	rq_list;	/* idle list */
	struct svc_sock *	rq_sock;	/* socket */
	struct sockaddr_in	rq_addr;	/* peer address */
	int			rq_addrlen;

	struct svc_serv *	rq_server;	/* RPC service definition */
	struct svc_procedure *	rq_procinfo;	/* procedure info */
	struct auth_ops *	rq_authop;	/* authentication flavour */
	struct svc_cred		rq_cred;	/* auth info */
	struct sk_buff *	rq_skbuff;	/* fast recv inet buffer */
	struct svc_deferred_req*rq_deferred;	/* deferred request we are replaying */

	struct xdr_buf		rq_arg;
	struct xdr_buf		rq_res;
	struct page *		rq_argpages[RPCSVC_MAXPAGES];
	struct page *		rq_respages[RPCSVC_MAXPAGES];
	int			rq_restailpage;
	short			rq_argused;	/* pages used for argument */
	short			rq_arghi;	/* pages available in argument page list */
	short			rq_resused;	/* pages used for result */

	u32			rq_xid;		/* transmission id */
	u32			rq_prog;	/* program number */
	u32			rq_vers;	/* program version */
	u32			rq_proc;	/* procedure number */
	u32			rq_prot;	/* IP protocol */
	unsigned short
				rq_secure  : 1;	/* secure port */


	__u32			rq_daddr;	/* dest addr of request - reply from here */

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
	struct svc_cacherep *	rq_cacherep;	/* cache info */
	struct knfsd_fh *	rq_reffh;	/* Referrence filehandle, used to
						 * determine what device number
						 * to report (real or virtual)
						 */
	int			rq_sendfile_ok; /* turned off in gss privacy
						 * to prevent encrypting page
						 * cache pages */
	wait_queue_head_t	rq_wait;	/* synchronization */
};

static inline int
xdr_argsize_check(struct svc_rqst *rqstp, u32 *p)
{
	char *cp = (char *)p;
	struct kvec *vec = &rqstp->rq_arg.head[0];
	return cp >= (char*)vec->iov_base
		&& cp <= (char*)vec->iov_base + vec->iov_len;
}

static inline int
xdr_ressize_check(struct svc_rqst *rqstp, u32 *p)
{
	struct kvec *vec = &rqstp->rq_res.head[0];
	char *cp = (char*)p;

	vec->iov_len = cp - (char*)vec->iov_base;

	return vec->iov_len <= PAGE_SIZE;
}

static inline struct page *
svc_take_res_page(struct svc_rqst *rqstp)
{
	if (rqstp->rq_arghi <= rqstp->rq_argused)
		return NULL;
	rqstp->rq_arghi--;
	rqstp->rq_respages[rqstp->rq_resused] =
		rqstp->rq_argpages[rqstp->rq_arghi];
	return rqstp->rq_respages[rqstp->rq_resused++];
}

static inline void svc_take_page(struct svc_rqst *rqstp)
{
	if (rqstp->rq_arghi <= rqstp->rq_argused) {
		WARN_ON(1);
		return;
	}
	rqstp->rq_arghi--;
	rqstp->rq_respages[rqstp->rq_resused] =
		rqstp->rq_argpages[rqstp->rq_arghi];
	rqstp->rq_resused++;
}

static inline void svc_pushback_allpages(struct svc_rqst *rqstp)
{
        while (rqstp->rq_resused) {
		if (rqstp->rq_respages[--rqstp->rq_resused] == NULL)
			continue;
		rqstp->rq_argpages[rqstp->rq_arghi++] =
			rqstp->rq_respages[rqstp->rq_resused];
		rqstp->rq_respages[rqstp->rq_resused] = NULL;
	}
}

static inline void svc_pushback_unused_pages(struct svc_rqst *rqstp)
{
	while (rqstp->rq_resused &&
	       rqstp->rq_res.pages != &rqstp->rq_respages[rqstp->rq_resused]) {

		if (rqstp->rq_respages[--rqstp->rq_resused] != NULL) {
			rqstp->rq_argpages[rqstp->rq_arghi++] =
				rqstp->rq_respages[rqstp->rq_resused];
			rqstp->rq_respages[rqstp->rq_resused] = NULL;
		}
	}
}

static inline void svc_free_allpages(struct svc_rqst *rqstp)
{
        while (rqstp->rq_resused) {
		if (rqstp->rq_respages[--rqstp->rq_resused] == NULL)
			continue;
		put_page(rqstp->rq_respages[rqstp->rq_resused]);
		rqstp->rq_respages[rqstp->rq_resused] = NULL;
	}
}

struct svc_deferred_req {
	u32			prot;	/* protocol (UDP or TCP) */
	struct sockaddr_in	addr;
	struct svc_sock		*svsk;	/* where reply must go */
	u32			daddr;	/* where reply must come from */
	struct cache_deferred_req handle;
	int			argslen;
	u32			args[0];
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

	/* Override dispatch function (e.g. when caching replies).
	 * A return value of 0 means drop the request. 
	 * vs_dispatch == NULL means use default dispatcher.
	 */
	int			(*vs_dispatch)(struct svc_rqst *, u32 *);
};

typedef int	(*svc_procfunc)(struct svc_rqst *, void *argp, void *resp);
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

typedef void		(*svc_thread_fn)(struct svc_rqst *);

struct svc_serv *  svc_create(struct svc_program *, unsigned int);
int		   svc_create_thread(svc_thread_fn, struct svc_serv *);
void		   svc_exit_thread(struct svc_rqst *);
void		   svc_destroy(struct svc_serv *);
int		   svc_process(struct svc_serv *, struct svc_rqst *);
int		   svc_register(struct svc_serv *, int, unsigned short);
void		   svc_wake_up(struct svc_serv *);
void		   svc_reserve(struct svc_rqst *rqstp, int space);

#endif /* SUNRPC_SVC_H */
