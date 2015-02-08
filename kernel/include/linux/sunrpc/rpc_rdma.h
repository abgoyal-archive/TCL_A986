

#ifndef _LINUX_SUNRPC_RPC_RDMA_H
#define _LINUX_SUNRPC_RPC_RDMA_H

#include <linux/types.h>

struct rpcrdma_segment {
	__be32 rs_handle;	/* Registered memory handle */
	__be32 rs_length;	/* Length of the chunk in bytes */
	__be64 rs_offset;	/* Chunk virtual address or offset */
};

struct rpcrdma_read_chunk {
	__be32 rc_discrim;	/* 1 indicates presence */
	__be32 rc_position;	/* Position in XDR stream */
	struct rpcrdma_segment rc_target;
};

struct rpcrdma_write_chunk {
	struct rpcrdma_segment wc_target;
};

struct rpcrdma_write_array {
	__be32 wc_discrim;	/* 1 indicates presence */
	__be32 wc_nchunks;	/* Array count */
	struct rpcrdma_write_chunk wc_array[0];
};

struct rpcrdma_msg {
	__be32 rm_xid;	/* Mirrors the RPC header xid */
	__be32 rm_vers;	/* Version of this protocol */
	__be32 rm_credit;	/* Buffers requested/granted */
	__be32 rm_type;	/* Type of message (enum rpcrdma_proc) */
	union {

		struct {			/* no chunks */
			__be32 rm_empty[3];	/* 3 empty chunk lists */
		} rm_nochunks;

		struct {			/* no chunks and padded */
			__be32 rm_align;	/* Padding alignment */
			__be32 rm_thresh;	/* Padding threshold */
			__be32 rm_pempty[3];	/* 3 empty chunk lists */
		} rm_padded;

		__be32 rm_chunks[0];	/* read, write and reply chunks */

	} rm_body;
};

#define RPCRDMA_HDRLEN_MIN	28

enum rpcrdma_errcode {
	ERR_VERS = 1,
	ERR_CHUNK = 2
};

struct rpcrdma_err_vers {
	uint32_t rdma_vers_low;	/* Version range supported by peer */
	uint32_t rdma_vers_high;
};

enum rpcrdma_proc {
	RDMA_MSG = 0,		/* An RPC call or reply msg */
	RDMA_NOMSG = 1,		/* An RPC call or reply msg - separate body */
	RDMA_MSGP = 2,		/* An RPC call or reply msg with padding */
	RDMA_DONE = 3,		/* Client signals reply completion */
	RDMA_ERROR = 4		/* An RPC RDMA encoding error */
};

#endif				/* _LINUX_SUNRPC_RPC_RDMA_H */
