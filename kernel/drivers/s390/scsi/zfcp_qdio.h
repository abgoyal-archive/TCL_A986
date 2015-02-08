

#ifndef ZFCP_QDIO_H
#define ZFCP_QDIO_H

#include <asm/qdio.h>

#define ZFCP_QDIO_SBALE_LEN	PAGE_SIZE

/* DMQ bug workaround: don't use last SBALE */
#define ZFCP_QDIO_MAX_SBALES_PER_SBAL	(QDIO_MAX_ELEMENTS_PER_BUFFER - 1)

/* index of last SBALE (with respect to DMQ bug workaround) */
#define ZFCP_QDIO_LAST_SBALE_PER_SBAL	(ZFCP_QDIO_MAX_SBALES_PER_SBAL - 1)

struct zfcp_qdio_queue {
	struct qdio_buffer *sbal[QDIO_MAX_BUFFERS_PER_Q];
	u8		   first;
	atomic_t           count;
};

struct zfcp_qdio {
	struct zfcp_qdio_queue	resp_q;
	struct zfcp_qdio_queue	req_q;
	spinlock_t		stat_lock;
	spinlock_t		req_q_lock;
	unsigned long long	req_q_time;
	u64			req_q_util;
	atomic_t		req_q_full;
	wait_queue_head_t	req_q_wq;
	struct zfcp_adapter	*adapter;
};

struct zfcp_qdio_req {
	u32	sbtype;
	u8	sbal_number;
	u8	sbal_first;
	u8	sbal_last;
	u8	sbal_limit;
	u8	sbale_curr;
	u8	sbal_response;
	u16	qdio_outb_usage;
	u16	qdio_inb_usage;
};

static inline struct qdio_buffer_element *
zfcp_qdio_sbale(struct zfcp_qdio_queue *q, int sbal_idx, int sbale_idx)
{
	return &q->sbal[sbal_idx]->element[sbale_idx];
}

static inline struct qdio_buffer_element *
zfcp_qdio_sbale_req(struct zfcp_qdio *qdio, struct zfcp_qdio_req *q_req)
{
	return zfcp_qdio_sbale(&qdio->req_q, q_req->sbal_last, 0);
}

static inline struct qdio_buffer_element *
zfcp_qdio_sbale_curr(struct zfcp_qdio *qdio, struct zfcp_qdio_req *q_req)
{
	return zfcp_qdio_sbale(&qdio->req_q, q_req->sbal_last,
			       q_req->sbale_curr);
}

static inline
void zfcp_qdio_req_init(struct zfcp_qdio *qdio, struct zfcp_qdio_req *q_req,
			unsigned long req_id, u32 sbtype, void *data, u32 len)
{
	struct qdio_buffer_element *sbale;

	q_req->sbal_first = q_req->sbal_last = qdio->req_q.first;
	q_req->sbal_number = 1;
	q_req->sbtype = sbtype;

	sbale = zfcp_qdio_sbale_req(qdio, q_req);
	sbale->addr = (void *) req_id;
	sbale->flags |= SBAL_FLAGS0_COMMAND;
	sbale->flags |= sbtype;

	q_req->sbale_curr = 1;
	sbale++;
	sbale->addr = data;
	if (likely(data))
		sbale->length = len;
}

static inline
void zfcp_qdio_fill_next(struct zfcp_qdio *qdio, struct zfcp_qdio_req *q_req,
			 void *data, u32 len)
{
	struct qdio_buffer_element *sbale;

	BUG_ON(q_req->sbale_curr == ZFCP_QDIO_LAST_SBALE_PER_SBAL);
	q_req->sbale_curr++;
	sbale = zfcp_qdio_sbale_curr(qdio, q_req);
	sbale->addr = data;
	sbale->length = len;
}

static inline
void zfcp_qdio_set_sbale_last(struct zfcp_qdio *qdio,
			      struct zfcp_qdio_req *q_req)
{
	struct qdio_buffer_element *sbale;

	sbale = zfcp_qdio_sbale_curr(qdio, q_req);
	sbale->flags |= SBAL_FLAGS_LAST_ENTRY;
}

static inline
int zfcp_qdio_sg_one_sbale(struct scatterlist *sg)
{
	return sg_is_last(sg) && sg->length <= ZFCP_QDIO_SBALE_LEN;
}

static inline
void zfcp_qdio_skip_to_last_sbale(struct zfcp_qdio_req *q_req)
{
	q_req->sbale_curr = ZFCP_QDIO_LAST_SBALE_PER_SBAL;
}

#endif /* ZFCP_QDIO_H */
