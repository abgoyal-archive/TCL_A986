


#ifndef _NET_SUNRPC_SUNRPC_H
#define _NET_SUNRPC_SUNRPC_H

#include <linux/net.h>

struct rpc_buffer {
	size_t	len;
	char	data[];
};

static inline int rpc_reply_expected(struct rpc_task *task)
{
	return (task->tk_msg.rpc_proc != NULL) &&
		(task->tk_msg.rpc_proc->p_decode != NULL);
}

int svc_send_common(struct socket *sock, struct xdr_buf *xdr,
		    struct page *headpage, unsigned long headoffset,
		    struct page *tailpage, unsigned long tailoffset);

#endif /* _NET_SUNRPC_SUNRPC_H */

