


#ifndef _LINUX_SUNRPC_BC_XPRT_H
#define _LINUX_SUNRPC_BC_XPRT_H

#include <linux/sunrpc/svcsock.h>
#include <linux/sunrpc/xprt.h>
#include <linux/sunrpc/sched.h>

#ifdef CONFIG_NFS_V4_1
struct rpc_rqst *xprt_alloc_bc_request(struct rpc_xprt *xprt);
void xprt_free_bc_request(struct rpc_rqst *req);
int xprt_setup_backchannel(struct rpc_xprt *, unsigned int min_reqs);
void xprt_destroy_backchannel(struct rpc_xprt *, int max_reqs);
int bc_send(struct rpc_rqst *req);

static inline int svc_is_backchannel(const struct svc_rqst *rqstp)
{
	if (rqstp->rq_server->bc_xprt)
		return 1;
	return 0;
}
#else /* CONFIG_NFS_V4_1 */
static inline int xprt_setup_backchannel(struct rpc_xprt *xprt,
					 unsigned int min_reqs)
{
	return 0;
}

static inline int svc_is_backchannel(const struct svc_rqst *rqstp)
{
	return 0;
}

static inline void xprt_free_bc_request(struct rpc_rqst *req)
{
}
#endif /* CONFIG_NFS_V4_1 */
#endif /* _LINUX_SUNRPC_BC_XPRT_H */

