
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/pci.h>
#include <linux/dma-mapping.h>
#include <rdma/rdma_cm.h>

#include "rds.h"
#include "ib.h"

static struct kmem_cache *rds_ib_incoming_slab;
static struct kmem_cache *rds_ib_frag_slab;
static atomic_t	rds_ib_allocation = ATOMIC_INIT(0);

static void rds_ib_frag_drop_page(struct rds_page_frag *frag)
{
	rdsdebug("frag %p page %p\n", frag, frag->f_page);
	__free_page(frag->f_page);
	frag->f_page = NULL;
}

static void rds_ib_frag_free(struct rds_page_frag *frag)
{
	rdsdebug("frag %p page %p\n", frag, frag->f_page);
	BUG_ON(frag->f_page != NULL);
	kmem_cache_free(rds_ib_frag_slab, frag);
}

static void rds_ib_recv_unmap_page(struct rds_ib_connection *ic,
				   struct rds_ib_recv_work *recv)
{
	struct rds_page_frag *frag = recv->r_frag;

	rdsdebug("recv %p frag %p page %p\n", recv, frag, frag->f_page);
	if (frag->f_mapped)
		ib_dma_unmap_page(ic->i_cm_id->device,
			       frag->f_mapped,
			       RDS_FRAG_SIZE, DMA_FROM_DEVICE);
	frag->f_mapped = 0;
}

void rds_ib_recv_init_ring(struct rds_ib_connection *ic)
{
	struct rds_ib_recv_work *recv;
	u32 i;

	for (i = 0, recv = ic->i_recvs; i < ic->i_recv_ring.w_nr; i++, recv++) {
		struct ib_sge *sge;

		recv->r_ibinc = NULL;
		recv->r_frag = NULL;

		recv->r_wr.next = NULL;
		recv->r_wr.wr_id = i;
		recv->r_wr.sg_list = recv->r_sge;
		recv->r_wr.num_sge = RDS_IB_RECV_SGE;

		sge = rds_ib_data_sge(ic, recv->r_sge);
		sge->addr = 0;
		sge->length = RDS_FRAG_SIZE;
		sge->lkey = ic->i_mr->lkey;

		sge = rds_ib_header_sge(ic, recv->r_sge);
		sge->addr = ic->i_recv_hdrs_dma + (i * sizeof(struct rds_header));
		sge->length = sizeof(struct rds_header);
		sge->lkey = ic->i_mr->lkey;
	}
}

static void rds_ib_recv_clear_one(struct rds_ib_connection *ic,
				  struct rds_ib_recv_work *recv)
{
	if (recv->r_ibinc) {
		rds_inc_put(&recv->r_ibinc->ii_inc);
		recv->r_ibinc = NULL;
	}
	if (recv->r_frag) {
		rds_ib_recv_unmap_page(ic, recv);
		if (recv->r_frag->f_page)
			rds_ib_frag_drop_page(recv->r_frag);
		rds_ib_frag_free(recv->r_frag);
		recv->r_frag = NULL;
	}
}

void rds_ib_recv_clear_ring(struct rds_ib_connection *ic)
{
	u32 i;

	for (i = 0; i < ic->i_recv_ring.w_nr; i++)
		rds_ib_recv_clear_one(ic, &ic->i_recvs[i]);

	if (ic->i_frag.f_page)
		rds_ib_frag_drop_page(&ic->i_frag);
}

static int rds_ib_recv_refill_one(struct rds_connection *conn,
				  struct rds_ib_recv_work *recv,
				  gfp_t kptr_gfp, gfp_t page_gfp)
{
	struct rds_ib_connection *ic = conn->c_transport_data;
	dma_addr_t dma_addr;
	struct ib_sge *sge;
	int ret = -ENOMEM;

	if (recv->r_ibinc == NULL) {
		if (!atomic_add_unless(&rds_ib_allocation, 1, rds_ib_sysctl_max_recv_allocation)) {
			rds_ib_stats_inc(s_ib_rx_alloc_limit);
			goto out;
		}
		recv->r_ibinc = kmem_cache_alloc(rds_ib_incoming_slab,
						 kptr_gfp);
		if (recv->r_ibinc == NULL) {
			atomic_dec(&rds_ib_allocation);
			goto out;
		}
		INIT_LIST_HEAD(&recv->r_ibinc->ii_frags);
		rds_inc_init(&recv->r_ibinc->ii_inc, conn, conn->c_faddr);
	}

	if (recv->r_frag == NULL) {
		recv->r_frag = kmem_cache_alloc(rds_ib_frag_slab, kptr_gfp);
		if (recv->r_frag == NULL)
			goto out;
		INIT_LIST_HEAD(&recv->r_frag->f_item);
		recv->r_frag->f_page = NULL;
	}

	if (ic->i_frag.f_page == NULL) {
		ic->i_frag.f_page = alloc_page(page_gfp);
		if (ic->i_frag.f_page == NULL)
			goto out;
		ic->i_frag.f_offset = 0;
	}

	dma_addr = ib_dma_map_page(ic->i_cm_id->device,
				  ic->i_frag.f_page,
				  ic->i_frag.f_offset,
				  RDS_FRAG_SIZE,
				  DMA_FROM_DEVICE);
	if (ib_dma_mapping_error(ic->i_cm_id->device, dma_addr))
		goto out;

	/*
	 * Once we get the RDS_PAGE_LAST_OFF frag then rds_ib_frag_unmap()
	 * must be called on this recv.  This happens as completions hit
	 * in order or on connection shutdown.
	 */
	recv->r_frag->f_page = ic->i_frag.f_page;
	recv->r_frag->f_offset = ic->i_frag.f_offset;
	recv->r_frag->f_mapped = dma_addr;

	sge = rds_ib_data_sge(ic, recv->r_sge);
	sge->addr = dma_addr;
	sge->length = RDS_FRAG_SIZE;

	sge = rds_ib_header_sge(ic, recv->r_sge);
	sge->addr = ic->i_recv_hdrs_dma + (recv - ic->i_recvs) * sizeof(struct rds_header);
	sge->length = sizeof(struct rds_header);

	get_page(recv->r_frag->f_page);

	if (ic->i_frag.f_offset < RDS_PAGE_LAST_OFF) {
		ic->i_frag.f_offset += RDS_FRAG_SIZE;
	} else {
		put_page(ic->i_frag.f_page);
		ic->i_frag.f_page = NULL;
		ic->i_frag.f_offset = 0;
	}

	ret = 0;
out:
	return ret;
}

int rds_ib_recv_refill(struct rds_connection *conn, gfp_t kptr_gfp,
		       gfp_t page_gfp, int prefill)
{
	struct rds_ib_connection *ic = conn->c_transport_data;
	struct rds_ib_recv_work *recv;
	struct ib_recv_wr *failed_wr;
	unsigned int posted = 0;
	int ret = 0;
	u32 pos;

	while ((prefill || rds_conn_up(conn)) &&
	       rds_ib_ring_alloc(&ic->i_recv_ring, 1, &pos)) {
		if (pos >= ic->i_recv_ring.w_nr) {
			printk(KERN_NOTICE "Argh - ring alloc returned pos=%u\n",
					pos);
			ret = -EINVAL;
			break;
		}

		recv = &ic->i_recvs[pos];
		ret = rds_ib_recv_refill_one(conn, recv, kptr_gfp, page_gfp);
		if (ret) {
			ret = -1;
			break;
		}

		/* XXX when can this fail? */
		ret = ib_post_recv(ic->i_cm_id->qp, &recv->r_wr, &failed_wr);
		rdsdebug("recv %p ibinc %p page %p addr %lu ret %d\n", recv,
			 recv->r_ibinc, recv->r_frag->f_page,
			 (long) recv->r_frag->f_mapped, ret);
		if (ret) {
			rds_ib_conn_error(conn, "recv post on "
			       "%pI4 returned %d, disconnecting and "
			       "reconnecting\n", &conn->c_faddr,
			       ret);
			ret = -1;
			break;
		}

		posted++;
	}

	/* We're doing flow control - update the window. */
	if (ic->i_flowctl && posted)
		rds_ib_advertise_credits(conn, posted);

	if (ret)
		rds_ib_ring_unalloc(&ic->i_recv_ring, 1);
	return ret;
}

void rds_ib_inc_purge(struct rds_incoming *inc)
{
	struct rds_ib_incoming *ibinc;
	struct rds_page_frag *frag;
	struct rds_page_frag *pos;

	ibinc = container_of(inc, struct rds_ib_incoming, ii_inc);
	rdsdebug("purging ibinc %p inc %p\n", ibinc, inc);

	list_for_each_entry_safe(frag, pos, &ibinc->ii_frags, f_item) {
		list_del_init(&frag->f_item);
		rds_ib_frag_drop_page(frag);
		rds_ib_frag_free(frag);
	}
}

void rds_ib_inc_free(struct rds_incoming *inc)
{
	struct rds_ib_incoming *ibinc;

	ibinc = container_of(inc, struct rds_ib_incoming, ii_inc);

	rds_ib_inc_purge(inc);
	rdsdebug("freeing ibinc %p inc %p\n", ibinc, inc);
	BUG_ON(!list_empty(&ibinc->ii_frags));
	kmem_cache_free(rds_ib_incoming_slab, ibinc);
	atomic_dec(&rds_ib_allocation);
	BUG_ON(atomic_read(&rds_ib_allocation) < 0);
}

int rds_ib_inc_copy_to_user(struct rds_incoming *inc, struct iovec *first_iov,
			    size_t size)
{
	struct rds_ib_incoming *ibinc;
	struct rds_page_frag *frag;
	struct iovec *iov = first_iov;
	unsigned long to_copy;
	unsigned long frag_off = 0;
	unsigned long iov_off = 0;
	int copied = 0;
	int ret;
	u32 len;

	ibinc = container_of(inc, struct rds_ib_incoming, ii_inc);
	frag = list_entry(ibinc->ii_frags.next, struct rds_page_frag, f_item);
	len = be32_to_cpu(inc->i_hdr.h_len);

	while (copied < size && copied < len) {
		if (frag_off == RDS_FRAG_SIZE) {
			frag = list_entry(frag->f_item.next,
					  struct rds_page_frag, f_item);
			frag_off = 0;
		}
		while (iov_off == iov->iov_len) {
			iov_off = 0;
			iov++;
		}

		to_copy = min(iov->iov_len - iov_off, RDS_FRAG_SIZE - frag_off);
		to_copy = min_t(size_t, to_copy, size - copied);
		to_copy = min_t(unsigned long, to_copy, len - copied);

		rdsdebug("%lu bytes to user [%p, %zu] + %lu from frag "
			 "[%p, %lu] + %lu\n",
			 to_copy, iov->iov_base, iov->iov_len, iov_off,
			 frag->f_page, frag->f_offset, frag_off);

		/* XXX needs + offset for multiple recvs per page */
		ret = rds_page_copy_to_user(frag->f_page,
					    frag->f_offset + frag_off,
					    iov->iov_base + iov_off,
					    to_copy);
		if (ret) {
			copied = ret;
			break;
		}

		iov_off += to_copy;
		frag_off += to_copy;
		copied += to_copy;
	}

	return copied;
}

/* ic starts out kzalloc()ed */
void rds_ib_recv_init_ack(struct rds_ib_connection *ic)
{
	struct ib_send_wr *wr = &ic->i_ack_wr;
	struct ib_sge *sge = &ic->i_ack_sge;

	sge->addr = ic->i_ack_dma;
	sge->length = sizeof(struct rds_header);
	sge->lkey = ic->i_mr->lkey;

	wr->sg_list = sge;
	wr->num_sge = 1;
	wr->opcode = IB_WR_SEND;
	wr->wr_id = RDS_IB_ACK_WR_ID;
	wr->send_flags = IB_SEND_SIGNALED | IB_SEND_SOLICITED;
}

#ifndef KERNEL_HAS_ATOMIC64
static void rds_ib_set_ack(struct rds_ib_connection *ic, u64 seq,
				int ack_required)
{
	unsigned long flags;

	spin_lock_irqsave(&ic->i_ack_lock, flags);
	ic->i_ack_next = seq;
	if (ack_required)
		set_bit(IB_ACK_REQUESTED, &ic->i_ack_flags);
	spin_unlock_irqrestore(&ic->i_ack_lock, flags);
}

static u64 rds_ib_get_ack(struct rds_ib_connection *ic)
{
	unsigned long flags;
	u64 seq;

	clear_bit(IB_ACK_REQUESTED, &ic->i_ack_flags);

	spin_lock_irqsave(&ic->i_ack_lock, flags);
	seq = ic->i_ack_next;
	spin_unlock_irqrestore(&ic->i_ack_lock, flags);

	return seq;
}
#else
static void rds_ib_set_ack(struct rds_ib_connection *ic, u64 seq,
				int ack_required)
{
	atomic64_set(&ic->i_ack_next, seq);
	if (ack_required) {
		smp_mb__before_clear_bit();
		set_bit(IB_ACK_REQUESTED, &ic->i_ack_flags);
	}
}

static u64 rds_ib_get_ack(struct rds_ib_connection *ic)
{
	clear_bit(IB_ACK_REQUESTED, &ic->i_ack_flags);
	smp_mb__after_clear_bit();

	return atomic64_read(&ic->i_ack_next);
}
#endif


static void rds_ib_send_ack(struct rds_ib_connection *ic, unsigned int adv_credits)
{
	struct rds_header *hdr = ic->i_ack;
	struct ib_send_wr *failed_wr;
	u64 seq;
	int ret;

	seq = rds_ib_get_ack(ic);

	rdsdebug("send_ack: ic %p ack %llu\n", ic, (unsigned long long) seq);
	rds_message_populate_header(hdr, 0, 0, 0);
	hdr->h_ack = cpu_to_be64(seq);
	hdr->h_credit = adv_credits;
	rds_message_make_checksum(hdr);
	ic->i_ack_queued = jiffies;

	ret = ib_post_send(ic->i_cm_id->qp, &ic->i_ack_wr, &failed_wr);
	if (unlikely(ret)) {
		/* Failed to send. Release the WR, and
		 * force another ACK.
		 */
		clear_bit(IB_ACK_IN_FLIGHT, &ic->i_ack_flags);
		set_bit(IB_ACK_REQUESTED, &ic->i_ack_flags);

		rds_ib_stats_inc(s_ib_ack_send_failure);

		rds_ib_conn_error(ic->conn, "sending ack failed\n");
	} else
		rds_ib_stats_inc(s_ib_ack_sent);
}


void rds_ib_attempt_ack(struct rds_ib_connection *ic)
{
	unsigned int adv_credits;

	if (!test_bit(IB_ACK_REQUESTED, &ic->i_ack_flags))
		return;

	if (test_and_set_bit(IB_ACK_IN_FLIGHT, &ic->i_ack_flags)) {
		rds_ib_stats_inc(s_ib_ack_send_delayed);
		return;
	}

	/* Can we get a send credit? */
	if (!rds_ib_send_grab_credits(ic, 1, &adv_credits, 0, RDS_MAX_ADV_CREDIT)) {
		rds_ib_stats_inc(s_ib_tx_throttle);
		clear_bit(IB_ACK_IN_FLIGHT, &ic->i_ack_flags);
		return;
	}

	clear_bit(IB_ACK_REQUESTED, &ic->i_ack_flags);
	rds_ib_send_ack(ic, adv_credits);
}

void rds_ib_ack_send_complete(struct rds_ib_connection *ic)
{
	clear_bit(IB_ACK_IN_FLIGHT, &ic->i_ack_flags);
	rds_ib_attempt_ack(ic);
}

u64 rds_ib_piggyb_ack(struct rds_ib_connection *ic)
{
	if (test_and_clear_bit(IB_ACK_REQUESTED, &ic->i_ack_flags))
		rds_ib_stats_inc(s_ib_ack_send_piggybacked);
	return rds_ib_get_ack(ic);
}

static struct rds_header *rds_ib_get_header(struct rds_connection *conn,
					    struct rds_ib_recv_work *recv,
					    u32 data_len)
{
	struct rds_ib_connection *ic = conn->c_transport_data;
	void *hdr_buff = &ic->i_recv_hdrs[recv - ic->i_recvs];
	void *addr;
	u32 misplaced_hdr_bytes;

	/*
	 * Support header at the front (RDS 3.1+) as well as header-at-end.
	 *
	 * Cases:
	 * 1) header all in header buff (great!)
	 * 2) header all in data page (copy all to header buff)
	 * 3) header split across hdr buf + data page
	 *    (move bit in hdr buff to end before copying other bit from data page)
	 */
	if (conn->c_version > RDS_PROTOCOL_3_0 || data_len == RDS_FRAG_SIZE)
	        return hdr_buff;

	if (data_len <= (RDS_FRAG_SIZE - sizeof(struct rds_header))) {
		addr = kmap_atomic(recv->r_frag->f_page, KM_SOFTIRQ0);
		memcpy(hdr_buff,
		       addr + recv->r_frag->f_offset + data_len,
		       sizeof(struct rds_header));
		kunmap_atomic(addr, KM_SOFTIRQ0);
		return hdr_buff;
	}

	misplaced_hdr_bytes = (sizeof(struct rds_header) - (RDS_FRAG_SIZE - data_len));

	memmove(hdr_buff + misplaced_hdr_bytes, hdr_buff, misplaced_hdr_bytes);

	addr = kmap_atomic(recv->r_frag->f_page, KM_SOFTIRQ0);
	memcpy(hdr_buff, addr + recv->r_frag->f_offset + data_len,
	       sizeof(struct rds_header) - misplaced_hdr_bytes);
	kunmap_atomic(addr, KM_SOFTIRQ0);
	return hdr_buff;
}

static void rds_ib_cong_recv(struct rds_connection *conn,
			      struct rds_ib_incoming *ibinc)
{
	struct rds_cong_map *map;
	unsigned int map_off;
	unsigned int map_page;
	struct rds_page_frag *frag;
	unsigned long frag_off;
	unsigned long to_copy;
	unsigned long copied;
	uint64_t uncongested = 0;
	void *addr;

	/* catch completely corrupt packets */
	if (be32_to_cpu(ibinc->ii_inc.i_hdr.h_len) != RDS_CONG_MAP_BYTES)
		return;

	map = conn->c_fcong;
	map_page = 0;
	map_off = 0;

	frag = list_entry(ibinc->ii_frags.next, struct rds_page_frag, f_item);
	frag_off = 0;

	copied = 0;

	while (copied < RDS_CONG_MAP_BYTES) {
		uint64_t *src, *dst;
		unsigned int k;

		to_copy = min(RDS_FRAG_SIZE - frag_off, PAGE_SIZE - map_off);
		BUG_ON(to_copy & 7); /* Must be 64bit aligned. */

		addr = kmap_atomic(frag->f_page, KM_SOFTIRQ0);

		src = addr + frag_off;
		dst = (void *)map->m_page_addrs[map_page] + map_off;
		for (k = 0; k < to_copy; k += 8) {
			/* Record ports that became uncongested, ie
			 * bits that changed from 0 to 1. */
			uncongested |= ~(*src) & *dst;
			*dst++ = *src++;
		}
		kunmap_atomic(addr, KM_SOFTIRQ0);

		copied += to_copy;

		map_off += to_copy;
		if (map_off == PAGE_SIZE) {
			map_off = 0;
			map_page++;
		}

		frag_off += to_copy;
		if (frag_off == RDS_FRAG_SIZE) {
			frag = list_entry(frag->f_item.next,
					  struct rds_page_frag, f_item);
			frag_off = 0;
		}
	}

	/* the congestion map is in little endian order */
	uncongested = le64_to_cpu(uncongested);

	rds_cong_map_updated(map, uncongested);
}

struct rds_ib_ack_state {
	u64		ack_next;
	u64		ack_recv;
	unsigned int	ack_required:1;
	unsigned int	ack_next_valid:1;
	unsigned int	ack_recv_valid:1;
};

static void rds_ib_process_recv(struct rds_connection *conn,
				struct rds_ib_recv_work *recv, u32 data_len,
				struct rds_ib_ack_state *state)
{
	struct rds_ib_connection *ic = conn->c_transport_data;
	struct rds_ib_incoming *ibinc = ic->i_ibinc;
	struct rds_header *ihdr, *hdr;

	/* XXX shut down the connection if port 0,0 are seen? */

	rdsdebug("ic %p ibinc %p recv %p byte len %u\n", ic, ibinc, recv,
		 data_len);

	if (data_len < sizeof(struct rds_header)) {
		rds_ib_conn_error(conn, "incoming message "
		       "from %pI4 didn't inclue a "
		       "header, disconnecting and "
		       "reconnecting\n",
		       &conn->c_faddr);
		return;
	}
	data_len -= sizeof(struct rds_header);

	ihdr = rds_ib_get_header(conn, recv, data_len);

	/* Validate the checksum. */
	if (!rds_message_verify_checksum(ihdr)) {
		rds_ib_conn_error(conn, "incoming message "
		       "from %pI4 has corrupted header - "
		       "forcing a reconnect\n",
		       &conn->c_faddr);
		rds_stats_inc(s_recv_drop_bad_checksum);
		return;
	}

	/* Process the ACK sequence which comes with every packet */
	state->ack_recv = be64_to_cpu(ihdr->h_ack);
	state->ack_recv_valid = 1;

	/* Process the credits update if there was one */
	if (ihdr->h_credit)
		rds_ib_send_add_credits(conn, ihdr->h_credit);

	if (ihdr->h_sport == 0 && ihdr->h_dport == 0 && data_len == 0) {
		/* This is an ACK-only packet. The fact that it gets
		 * special treatment here is that historically, ACKs
		 * were rather special beasts.
		 */
		rds_ib_stats_inc(s_ib_ack_received);

		/*
		 * Usually the frags make their way on to incs and are then freed as
		 * the inc is freed.  We don't go that route, so we have to drop the
		 * page ref ourselves.  We can't just leave the page on the recv
		 * because that confuses the dma mapping of pages and each recv's use
		 * of a partial page.  We can leave the frag, though, it will be
		 * reused.
		 *
		 * FIXME: Fold this into the code path below.
		 */
		rds_ib_frag_drop_page(recv->r_frag);
		return;
	}

	/*
	 * If we don't already have an inc on the connection then this
	 * fragment has a header and starts a message.. copy its header
	 * into the inc and save the inc so we can hang upcoming fragments
	 * off its list.
	 */
	if (ibinc == NULL) {
		ibinc = recv->r_ibinc;
		recv->r_ibinc = NULL;
		ic->i_ibinc = ibinc;

		hdr = &ibinc->ii_inc.i_hdr;
		memcpy(hdr, ihdr, sizeof(*hdr));
		ic->i_recv_data_rem = be32_to_cpu(hdr->h_len);

		rdsdebug("ic %p ibinc %p rem %u flag 0x%x\n", ic, ibinc,
			 ic->i_recv_data_rem, hdr->h_flags);
	} else {
		hdr = &ibinc->ii_inc.i_hdr;
		/* We can't just use memcmp here; fragments of a
		 * single message may carry different ACKs */
		if (hdr->h_sequence != ihdr->h_sequence ||
		    hdr->h_len != ihdr->h_len ||
		    hdr->h_sport != ihdr->h_sport ||
		    hdr->h_dport != ihdr->h_dport) {
			rds_ib_conn_error(conn,
				"fragment header mismatch; forcing reconnect\n");
			return;
		}
	}

	list_add_tail(&recv->r_frag->f_item, &ibinc->ii_frags);
	recv->r_frag = NULL;

	if (ic->i_recv_data_rem > RDS_FRAG_SIZE)
		ic->i_recv_data_rem -= RDS_FRAG_SIZE;
	else {
		ic->i_recv_data_rem = 0;
		ic->i_ibinc = NULL;

		if (ibinc->ii_inc.i_hdr.h_flags == RDS_FLAG_CONG_BITMAP)
			rds_ib_cong_recv(conn, ibinc);
		else {
			rds_recv_incoming(conn, conn->c_faddr, conn->c_laddr,
					  &ibinc->ii_inc, GFP_ATOMIC,
					  KM_SOFTIRQ0);
			state->ack_next = be64_to_cpu(hdr->h_sequence);
			state->ack_next_valid = 1;
		}

		/* Evaluate the ACK_REQUIRED flag *after* we received
		 * the complete frame, and after bumping the next_rx
		 * sequence. */
		if (hdr->h_flags & RDS_FLAG_ACK_REQUIRED) {
			rds_stats_inc(s_recv_ack_required);
			state->ack_required = 1;
		}

		rds_inc_put(&ibinc->ii_inc);
	}
}

void rds_ib_recv_cq_comp_handler(struct ib_cq *cq, void *context)
{
	struct rds_connection *conn = context;
	struct rds_ib_connection *ic = conn->c_transport_data;

	rdsdebug("conn %p cq %p\n", conn, cq);

	rds_ib_stats_inc(s_ib_rx_cq_call);

	tasklet_schedule(&ic->i_recv_tasklet);
}

static inline void rds_poll_cq(struct rds_ib_connection *ic,
			       struct rds_ib_ack_state *state)
{
	struct rds_connection *conn = ic->conn;
	struct ib_wc wc;
	struct rds_ib_recv_work *recv;

	while (ib_poll_cq(ic->i_recv_cq, 1, &wc) > 0) {
		rdsdebug("wc wr_id 0x%llx status %u byte_len %u imm_data %u\n",
			 (unsigned long long)wc.wr_id, wc.status, wc.byte_len,
			 be32_to_cpu(wc.ex.imm_data));
		rds_ib_stats_inc(s_ib_rx_cq_event);

		recv = &ic->i_recvs[rds_ib_ring_oldest(&ic->i_recv_ring)];

		rds_ib_recv_unmap_page(ic, recv);

		/*
		 * Also process recvs in connecting state because it is possible
		 * to get a recv completion _before_ the rdmacm ESTABLISHED
		 * event is processed.
		 */
		if (rds_conn_up(conn) || rds_conn_connecting(conn)) {
			/* We expect errors as the qp is drained during shutdown */
			if (wc.status == IB_WC_SUCCESS) {
				rds_ib_process_recv(conn, recv, wc.byte_len, state);
			} else {
				rds_ib_conn_error(conn, "recv completion on "
				       "%pI4 had status %u, disconnecting and "
				       "reconnecting\n", &conn->c_faddr,
				       wc.status);
			}
		}

		rds_ib_ring_free(&ic->i_recv_ring, 1);
	}
}

void rds_ib_recv_tasklet_fn(unsigned long data)
{
	struct rds_ib_connection *ic = (struct rds_ib_connection *) data;
	struct rds_connection *conn = ic->conn;
	struct rds_ib_ack_state state = { 0, };

	rds_poll_cq(ic, &state);
	ib_req_notify_cq(ic->i_recv_cq, IB_CQ_SOLICITED);
	rds_poll_cq(ic, &state);

	if (state.ack_next_valid)
		rds_ib_set_ack(ic, state.ack_next, state.ack_required);
	if (state.ack_recv_valid && state.ack_recv > ic->i_ack_recv) {
		rds_send_drop_acked(conn, state.ack_recv, NULL);
		ic->i_ack_recv = state.ack_recv;
	}
	if (rds_conn_up(conn))
		rds_ib_attempt_ack(ic);

	/* If we ever end up with a really empty receive ring, we're
	 * in deep trouble, as the sender will definitely see RNR
	 * timeouts. */
	if (rds_ib_ring_empty(&ic->i_recv_ring))
		rds_ib_stats_inc(s_ib_rx_ring_empty);

	/*
	 * If the ring is running low, then schedule the thread to refill.
	 */
	if (rds_ib_ring_low(&ic->i_recv_ring))
		queue_delayed_work(rds_wq, &conn->c_recv_w, 0);
}

int rds_ib_recv(struct rds_connection *conn)
{
	struct rds_ib_connection *ic = conn->c_transport_data;
	int ret = 0;

	rdsdebug("conn %p\n", conn);

	/*
	 * If we get a temporary posting failure in this context then
	 * we're really low and we want the caller to back off for a bit.
	 */
	mutex_lock(&ic->i_recv_mutex);
	if (rds_ib_recv_refill(conn, GFP_KERNEL, GFP_HIGHUSER, 0))
		ret = -ENOMEM;
	else
		rds_ib_stats_inc(s_ib_rx_refill_from_thread);
	mutex_unlock(&ic->i_recv_mutex);

	if (rds_conn_up(conn))
		rds_ib_attempt_ack(ic);

	return ret;
}

int __init rds_ib_recv_init(void)
{
	struct sysinfo si;
	int ret = -ENOMEM;

	/* Default to 30% of all available RAM for recv memory */
	si_meminfo(&si);
	rds_ib_sysctl_max_recv_allocation = si.totalram / 3 * PAGE_SIZE / RDS_FRAG_SIZE;

	rds_ib_incoming_slab = kmem_cache_create("rds_ib_incoming",
					sizeof(struct rds_ib_incoming),
					0, 0, NULL);
	if (rds_ib_incoming_slab == NULL)
		goto out;

	rds_ib_frag_slab = kmem_cache_create("rds_ib_frag",
					sizeof(struct rds_page_frag),
					0, 0, NULL);
	if (rds_ib_frag_slab == NULL)
		kmem_cache_destroy(rds_ib_incoming_slab);
	else
		ret = 0;
out:
	return ret;
}

void rds_ib_recv_exit(void)
{
	kmem_cache_destroy(rds_ib_incoming_slab);
	kmem_cache_destroy(rds_ib_frag_slab);
}
