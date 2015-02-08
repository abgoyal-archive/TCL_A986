

#ifndef C2_PROVIDER_H
#define C2_PROVIDER_H
#include <linux/inetdevice.h>

#include <rdma/ib_verbs.h>
#include <rdma/ib_pack.h>

#include "c2_mq.h"
#include <rdma/iw_cm.h>

#define C2_MPT_FLAG_ATOMIC        (1 << 14)
#define C2_MPT_FLAG_REMOTE_WRITE  (1 << 13)
#define C2_MPT_FLAG_REMOTE_READ   (1 << 12)
#define C2_MPT_FLAG_LOCAL_WRITE   (1 << 11)
#define C2_MPT_FLAG_LOCAL_READ    (1 << 10)

struct c2_buf_list {
	void *buf;
	DEFINE_DMA_UNMAP_ADDR(mapping);
};


struct c2_ucontext {
	struct ib_ucontext ibucontext;
};

struct c2_mtt;

struct c2_pd {
	struct ib_pd ibpd;
	u32 pd_id;
};

struct c2_mr {
	struct ib_mr ibmr;
	struct c2_pd *pd;
	struct ib_umem *umem;
};

struct c2_av;

enum c2_ah_type {
	C2_AH_ON_HCA,
	C2_AH_PCI_POOL,
	C2_AH_KMALLOC
};

struct c2_ah {
	struct ib_ah ibah;
};

struct c2_cq {
	struct ib_cq ibcq;
	spinlock_t lock;
	atomic_t refcount;
	int cqn;
	int is_kernel;
	wait_queue_head_t wait;

	u32 adapter_handle;
	struct c2_mq mq;
};

struct c2_wq {
	spinlock_t lock;
};
struct iw_cm_id;
struct c2_qp {
	struct ib_qp ibqp;
	struct iw_cm_id *cm_id;
	spinlock_t lock;
	atomic_t refcount;
	wait_queue_head_t wait;
	int qpn;

	u32 adapter_handle;
	u32 send_sgl_depth;
	u32 recv_sgl_depth;
	u32 rdma_write_sgl_depth;
	u8 state;

	struct c2_mq sq_mq;
	struct c2_mq rq_mq;
};

struct c2_cr_query_attrs {
	u32 local_addr;
	u32 remote_addr;
	u16 local_port;
	u16 remote_port;
};

static inline struct c2_pd *to_c2pd(struct ib_pd *ibpd)
{
	return container_of(ibpd, struct c2_pd, ibpd);
}

static inline struct c2_ucontext *to_c2ucontext(struct ib_ucontext *ibucontext)
{
	return container_of(ibucontext, struct c2_ucontext, ibucontext);
}

static inline struct c2_mr *to_c2mr(struct ib_mr *ibmr)
{
	return container_of(ibmr, struct c2_mr, ibmr);
}


static inline struct c2_ah *to_c2ah(struct ib_ah *ibah)
{
	return container_of(ibah, struct c2_ah, ibah);
}

static inline struct c2_cq *to_c2cq(struct ib_cq *ibcq)
{
	return container_of(ibcq, struct c2_cq, ibcq);
}

static inline struct c2_qp *to_c2qp(struct ib_qp *ibqp)
{
	return container_of(ibqp, struct c2_qp, ibqp);
}

static inline int is_rnic_addr(struct net_device *netdev, u32 addr)
{
	struct in_device *ind;
	int ret = 0;

	ind = in_dev_get(netdev);
	if (!ind)
		return 0;

	for_ifa(ind) {
		if (ifa->ifa_address == addr) {
			ret = 1;
			break;
		}
	}
	endfor_ifa(ind);
	in_dev_put(ind);
	return ret;
}
#endif				/* C2_PROVIDER_H */
