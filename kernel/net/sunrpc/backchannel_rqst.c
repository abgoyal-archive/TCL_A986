

#include <linux/tcp.h>
#include <linux/slab.h>
#include <linux/sunrpc/xprt.h>

#ifdef RPC_DEBUG
#define RPCDBG_FACILITY	RPCDBG_TRANS
#endif

#if defined(CONFIG_NFS_V4_1)

static inline int xprt_need_to_requeue(struct rpc_xprt *xprt)
{
	return xprt->bc_alloc_count > 0;
}

static inline void xprt_inc_alloc_count(struct rpc_xprt *xprt, unsigned int n)
{
	xprt->bc_alloc_count += n;
}

static inline int xprt_dec_alloc_count(struct rpc_xprt *xprt, unsigned int n)
{
	return xprt->bc_alloc_count -= n;
}

static void xprt_free_allocation(struct rpc_rqst *req)
{
	struct xdr_buf *xbufp;

	dprintk("RPC:        free allocations for req= %p\n", req);
	BUG_ON(test_bit(RPC_BC_PA_IN_USE, &req->rq_bc_pa_state));
	xbufp = &req->rq_private_buf;
	free_page((unsigned long)xbufp->head[0].iov_base);
	xbufp = &req->rq_snd_buf;
	free_page((unsigned long)xbufp->head[0].iov_base);
	list_del(&req->rq_bc_pa_list);
	kfree(req);
}

int xprt_setup_backchannel(struct rpc_xprt *xprt, unsigned int min_reqs)
{
	struct page *page_rcv = NULL, *page_snd = NULL;
	struct xdr_buf *xbufp = NULL;
	struct rpc_rqst *req, *tmp;
	struct list_head tmp_list;
	int i;

	dprintk("RPC:       setup backchannel transport\n");

	/*
	 * We use a temporary list to keep track of the preallocated
	 * buffers.  Once we're done building the list we splice it
	 * into the backchannel preallocation list off of the rpc_xprt
	 * struct.  This helps minimize the amount of time the list
	 * lock is held on the rpc_xprt struct.  It also makes cleanup
	 * easier in case of memory allocation errors.
	 */
	INIT_LIST_HEAD(&tmp_list);
	for (i = 0; i < min_reqs; i++) {
		/* Pre-allocate one backchannel rpc_rqst */
		req = kzalloc(sizeof(struct rpc_rqst), GFP_KERNEL);
		if (req == NULL) {
			printk(KERN_ERR "Failed to create bc rpc_rqst\n");
			goto out_free;
		}

		/* Add the allocated buffer to the tmp list */
		dprintk("RPC:       adding req= %p\n", req);
		list_add(&req->rq_bc_pa_list, &tmp_list);

		req->rq_xprt = xprt;
		INIT_LIST_HEAD(&req->rq_list);
		INIT_LIST_HEAD(&req->rq_bc_list);

		/* Preallocate one XDR receive buffer */
		page_rcv = alloc_page(GFP_KERNEL);
		if (page_rcv == NULL) {
			printk(KERN_ERR "Failed to create bc receive xbuf\n");
			goto out_free;
		}
		xbufp = &req->rq_rcv_buf;
		xbufp->head[0].iov_base = page_address(page_rcv);
		xbufp->head[0].iov_len = PAGE_SIZE;
		xbufp->tail[0].iov_base = NULL;
		xbufp->tail[0].iov_len = 0;
		xbufp->page_len = 0;
		xbufp->len = PAGE_SIZE;
		xbufp->buflen = PAGE_SIZE;

		/* Preallocate one XDR send buffer */
		page_snd = alloc_page(GFP_KERNEL);
		if (page_snd == NULL) {
			printk(KERN_ERR "Failed to create bc snd xbuf\n");
			goto out_free;
		}

		xbufp = &req->rq_snd_buf;
		xbufp->head[0].iov_base = page_address(page_snd);
		xbufp->head[0].iov_len = 0;
		xbufp->tail[0].iov_base = NULL;
		xbufp->tail[0].iov_len = 0;
		xbufp->page_len = 0;
		xbufp->len = 0;
		xbufp->buflen = PAGE_SIZE;
	}

	/*
	 * Add the temporary list to the backchannel preallocation list
	 */
	spin_lock_bh(&xprt->bc_pa_lock);
	list_splice(&tmp_list, &xprt->bc_pa_list);
	xprt_inc_alloc_count(xprt, min_reqs);
	spin_unlock_bh(&xprt->bc_pa_lock);

	dprintk("RPC:       setup backchannel transport done\n");
	return 0;

out_free:
	/*
	 * Memory allocation failed, free the temporary list
	 */
	list_for_each_entry_safe(req, tmp, &tmp_list, rq_bc_pa_list)
		xprt_free_allocation(req);

	dprintk("RPC:       setup backchannel transport failed\n");
	return -1;
}
EXPORT_SYMBOL(xprt_setup_backchannel);

void xprt_destroy_backchannel(struct rpc_xprt *xprt, unsigned int max_reqs)
{
	struct rpc_rqst *req = NULL, *tmp = NULL;

	dprintk("RPC:        destroy backchannel transport\n");

	BUG_ON(max_reqs == 0);
	spin_lock_bh(&xprt->bc_pa_lock);
	xprt_dec_alloc_count(xprt, max_reqs);
	list_for_each_entry_safe(req, tmp, &xprt->bc_pa_list, rq_bc_pa_list) {
		dprintk("RPC:        req=%p\n", req);
		xprt_free_allocation(req);
		if (--max_reqs == 0)
			break;
	}
	spin_unlock_bh(&xprt->bc_pa_lock);

	dprintk("RPC:        backchannel list empty= %s\n",
		list_empty(&xprt->bc_pa_list) ? "true" : "false");
}
EXPORT_SYMBOL(xprt_destroy_backchannel);

struct rpc_rqst *xprt_alloc_bc_request(struct rpc_xprt *xprt)
{
	struct rpc_rqst *req;

	dprintk("RPC:       allocate a backchannel request\n");
	spin_lock(&xprt->bc_pa_lock);
	if (!list_empty(&xprt->bc_pa_list)) {
		req = list_first_entry(&xprt->bc_pa_list, struct rpc_rqst,
				rq_bc_pa_list);
		list_del(&req->rq_bc_pa_list);
	} else {
		req = NULL;
	}
	spin_unlock(&xprt->bc_pa_lock);

	if (req != NULL) {
		set_bit(RPC_BC_PA_IN_USE, &req->rq_bc_pa_state);
		req->rq_reply_bytes_recvd = 0;
		req->rq_bytes_sent = 0;
		memcpy(&req->rq_private_buf, &req->rq_rcv_buf,
			sizeof(req->rq_private_buf));
	}
	dprintk("RPC:       backchannel req=%p\n", req);
	return req;
}

void xprt_free_bc_request(struct rpc_rqst *req)
{
	struct rpc_xprt *xprt = req->rq_xprt;

	dprintk("RPC:       free backchannel req=%p\n", req);

	smp_mb__before_clear_bit();
	BUG_ON(!test_bit(RPC_BC_PA_IN_USE, &req->rq_bc_pa_state));
	clear_bit(RPC_BC_PA_IN_USE, &req->rq_bc_pa_state);
	smp_mb__after_clear_bit();

	if (!xprt_need_to_requeue(xprt)) {
		/*
		 * The last remaining session was destroyed while this
		 * entry was in use.  Free the entry and don't attempt
		 * to add back to the list because there is no need to
		 * have anymore preallocated entries.
		 */
		dprintk("RPC:       Last session removed req=%p\n", req);
		xprt_free_allocation(req);
		return;
	}

	/*
	 * Return it to the list of preallocations so that it
	 * may be reused by a new callback request.
	 */
	spin_lock_bh(&xprt->bc_pa_lock);
	list_add(&req->rq_bc_pa_list, &xprt->bc_pa_list);
	spin_unlock_bh(&xprt->bc_pa_lock);
}

#endif /* CONFIG_NFS_V4_1 */
