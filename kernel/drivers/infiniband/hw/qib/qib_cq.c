

#include <linux/err.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>

#include "qib_verbs.h"

void qib_cq_enter(struct qib_cq *cq, struct ib_wc *entry, int solicited)
{
	struct qib_cq_wc *wc;
	unsigned long flags;
	u32 head;
	u32 next;

	spin_lock_irqsave(&cq->lock, flags);

	/*
	 * Note that the head pointer might be writable by user processes.
	 * Take care to verify it is a sane value.
	 */
	wc = cq->queue;
	head = wc->head;
	if (head >= (unsigned) cq->ibcq.cqe) {
		head = cq->ibcq.cqe;
		next = 0;
	} else
		next = head + 1;
	if (unlikely(next == wc->tail)) {
		spin_unlock_irqrestore(&cq->lock, flags);
		if (cq->ibcq.event_handler) {
			struct ib_event ev;

			ev.device = cq->ibcq.device;
			ev.element.cq = &cq->ibcq;
			ev.event = IB_EVENT_CQ_ERR;
			cq->ibcq.event_handler(&ev, cq->ibcq.cq_context);
		}
		return;
	}
	if (cq->ip) {
		wc->uqueue[head].wr_id = entry->wr_id;
		wc->uqueue[head].status = entry->status;
		wc->uqueue[head].opcode = entry->opcode;
		wc->uqueue[head].vendor_err = entry->vendor_err;
		wc->uqueue[head].byte_len = entry->byte_len;
		wc->uqueue[head].ex.imm_data =
			(__u32 __force)entry->ex.imm_data;
		wc->uqueue[head].qp_num = entry->qp->qp_num;
		wc->uqueue[head].src_qp = entry->src_qp;
		wc->uqueue[head].wc_flags = entry->wc_flags;
		wc->uqueue[head].pkey_index = entry->pkey_index;
		wc->uqueue[head].slid = entry->slid;
		wc->uqueue[head].sl = entry->sl;
		wc->uqueue[head].dlid_path_bits = entry->dlid_path_bits;
		wc->uqueue[head].port_num = entry->port_num;
		/* Make sure entry is written before the head index. */
		smp_wmb();
	} else
		wc->kqueue[head] = *entry;
	wc->head = next;

	if (cq->notify == IB_CQ_NEXT_COMP ||
	    (cq->notify == IB_CQ_SOLICITED && solicited)) {
		cq->notify = IB_CQ_NONE;
		cq->triggered++;
		/*
		 * This will cause send_complete() to be called in
		 * another thread.
		 */
		queue_work(qib_cq_wq, &cq->comptask);
	}

	spin_unlock_irqrestore(&cq->lock, flags);
}

int qib_poll_cq(struct ib_cq *ibcq, int num_entries, struct ib_wc *entry)
{
	struct qib_cq *cq = to_icq(ibcq);
	struct qib_cq_wc *wc;
	unsigned long flags;
	int npolled;
	u32 tail;

	/* The kernel can only poll a kernel completion queue */
	if (cq->ip) {
		npolled = -EINVAL;
		goto bail;
	}

	spin_lock_irqsave(&cq->lock, flags);

	wc = cq->queue;
	tail = wc->tail;
	if (tail > (u32) cq->ibcq.cqe)
		tail = (u32) cq->ibcq.cqe;
	for (npolled = 0; npolled < num_entries; ++npolled, ++entry) {
		if (tail == wc->head)
			break;
		/* The kernel doesn't need a RMB since it has the lock. */
		*entry = wc->kqueue[tail];
		if (tail >= cq->ibcq.cqe)
			tail = 0;
		else
			tail++;
	}
	wc->tail = tail;

	spin_unlock_irqrestore(&cq->lock, flags);

bail:
	return npolled;
}

static void send_complete(struct work_struct *work)
{
	struct qib_cq *cq = container_of(work, struct qib_cq, comptask);

	/*
	 * The completion handler will most likely rearm the notification
	 * and poll for all pending entries.  If a new completion entry
	 * is added while we are in this routine, queue_work()
	 * won't call us again until we return so we check triggered to
	 * see if we need to call the handler again.
	 */
	for (;;) {
		u8 triggered = cq->triggered;

		/*
		 * IPoIB connected mode assumes the callback is from a
		 * soft IRQ. We simulate this by blocking "bottom halves".
		 * See the implementation for ipoib_cm_handle_tx_wc(),
		 * netif_tx_lock_bh() and netif_tx_lock().
		 */
		local_bh_disable();
		cq->ibcq.comp_handler(&cq->ibcq, cq->ibcq.cq_context);
		local_bh_enable();

		if (cq->triggered == triggered)
			return;
	}
}

struct ib_cq *qib_create_cq(struct ib_device *ibdev, int entries,
			    int comp_vector, struct ib_ucontext *context,
			    struct ib_udata *udata)
{
	struct qib_ibdev *dev = to_idev(ibdev);
	struct qib_cq *cq;
	struct qib_cq_wc *wc;
	struct ib_cq *ret;
	u32 sz;

	if (entries < 1 || entries > ib_qib_max_cqes) {
		ret = ERR_PTR(-EINVAL);
		goto done;
	}

	/* Allocate the completion queue structure. */
	cq = kmalloc(sizeof(*cq), GFP_KERNEL);
	if (!cq) {
		ret = ERR_PTR(-ENOMEM);
		goto done;
	}

	/*
	 * Allocate the completion queue entries and head/tail pointers.
	 * This is allocated separately so that it can be resized and
	 * also mapped into user space.
	 * We need to use vmalloc() in order to support mmap and large
	 * numbers of entries.
	 */
	sz = sizeof(*wc);
	if (udata && udata->outlen >= sizeof(__u64))
		sz += sizeof(struct ib_uverbs_wc) * (entries + 1);
	else
		sz += sizeof(struct ib_wc) * (entries + 1);
	wc = vmalloc_user(sz);
	if (!wc) {
		ret = ERR_PTR(-ENOMEM);
		goto bail_cq;
	}

	/*
	 * Return the address of the WC as the offset to mmap.
	 * See qib_mmap() for details.
	 */
	if (udata && udata->outlen >= sizeof(__u64)) {
		int err;

		cq->ip = qib_create_mmap_info(dev, sz, context, wc);
		if (!cq->ip) {
			ret = ERR_PTR(-ENOMEM);
			goto bail_wc;
		}

		err = ib_copy_to_udata(udata, &cq->ip->offset,
				       sizeof(cq->ip->offset));
		if (err) {
			ret = ERR_PTR(err);
			goto bail_ip;
		}
	} else
		cq->ip = NULL;

	spin_lock(&dev->n_cqs_lock);
	if (dev->n_cqs_allocated == ib_qib_max_cqs) {
		spin_unlock(&dev->n_cqs_lock);
		ret = ERR_PTR(-ENOMEM);
		goto bail_ip;
	}

	dev->n_cqs_allocated++;
	spin_unlock(&dev->n_cqs_lock);

	if (cq->ip) {
		spin_lock_irq(&dev->pending_lock);
		list_add(&cq->ip->pending_mmaps, &dev->pending_mmaps);
		spin_unlock_irq(&dev->pending_lock);
	}

	/*
	 * ib_create_cq() will initialize cq->ibcq except for cq->ibcq.cqe.
	 * The number of entries should be >= the number requested or return
	 * an error.
	 */
	cq->ibcq.cqe = entries;
	cq->notify = IB_CQ_NONE;
	cq->triggered = 0;
	spin_lock_init(&cq->lock);
	INIT_WORK(&cq->comptask, send_complete);
	wc->head = 0;
	wc->tail = 0;
	cq->queue = wc;

	ret = &cq->ibcq;

	goto done;

bail_ip:
	kfree(cq->ip);
bail_wc:
	vfree(wc);
bail_cq:
	kfree(cq);
done:
	return ret;
}

int qib_destroy_cq(struct ib_cq *ibcq)
{
	struct qib_ibdev *dev = to_idev(ibcq->device);
	struct qib_cq *cq = to_icq(ibcq);

	flush_work(&cq->comptask);
	spin_lock(&dev->n_cqs_lock);
	dev->n_cqs_allocated--;
	spin_unlock(&dev->n_cqs_lock);
	if (cq->ip)
		kref_put(&cq->ip->ref, qib_release_mmap_info);
	else
		vfree(cq->queue);
	kfree(cq);

	return 0;
}

int qib_req_notify_cq(struct ib_cq *ibcq, enum ib_cq_notify_flags notify_flags)
{
	struct qib_cq *cq = to_icq(ibcq);
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&cq->lock, flags);
	/*
	 * Don't change IB_CQ_NEXT_COMP to IB_CQ_SOLICITED but allow
	 * any other transitions (see C11-31 and C11-32 in ch. 11.4.2.2).
	 */
	if (cq->notify != IB_CQ_NEXT_COMP)
		cq->notify = notify_flags & IB_CQ_SOLICITED_MASK;

	if ((notify_flags & IB_CQ_REPORT_MISSED_EVENTS) &&
	    cq->queue->head != cq->queue->tail)
		ret = 1;

	spin_unlock_irqrestore(&cq->lock, flags);

	return ret;
}

int qib_resize_cq(struct ib_cq *ibcq, int cqe, struct ib_udata *udata)
{
	struct qib_cq *cq = to_icq(ibcq);
	struct qib_cq_wc *old_wc;
	struct qib_cq_wc *wc;
	u32 head, tail, n;
	int ret;
	u32 sz;

	if (cqe < 1 || cqe > ib_qib_max_cqes) {
		ret = -EINVAL;
		goto bail;
	}

	/*
	 * Need to use vmalloc() if we want to support large #s of entries.
	 */
	sz = sizeof(*wc);
	if (udata && udata->outlen >= sizeof(__u64))
		sz += sizeof(struct ib_uverbs_wc) * (cqe + 1);
	else
		sz += sizeof(struct ib_wc) * (cqe + 1);
	wc = vmalloc_user(sz);
	if (!wc) {
		ret = -ENOMEM;
		goto bail;
	}

	/* Check that we can write the offset to mmap. */
	if (udata && udata->outlen >= sizeof(__u64)) {
		__u64 offset = 0;

		ret = ib_copy_to_udata(udata, &offset, sizeof(offset));
		if (ret)
			goto bail_free;
	}

	spin_lock_irq(&cq->lock);
	/*
	 * Make sure head and tail are sane since they
	 * might be user writable.
	 */
	old_wc = cq->queue;
	head = old_wc->head;
	if (head > (u32) cq->ibcq.cqe)
		head = (u32) cq->ibcq.cqe;
	tail = old_wc->tail;
	if (tail > (u32) cq->ibcq.cqe)
		tail = (u32) cq->ibcq.cqe;
	if (head < tail)
		n = cq->ibcq.cqe + 1 + head - tail;
	else
		n = head - tail;
	if (unlikely((u32)cqe < n)) {
		ret = -EINVAL;
		goto bail_unlock;
	}
	for (n = 0; tail != head; n++) {
		if (cq->ip)
			wc->uqueue[n] = old_wc->uqueue[tail];
		else
			wc->kqueue[n] = old_wc->kqueue[tail];
		if (tail == (u32) cq->ibcq.cqe)
			tail = 0;
		else
			tail++;
	}
	cq->ibcq.cqe = cqe;
	wc->head = n;
	wc->tail = 0;
	cq->queue = wc;
	spin_unlock_irq(&cq->lock);

	vfree(old_wc);

	if (cq->ip) {
		struct qib_ibdev *dev = to_idev(ibcq->device);
		struct qib_mmap_info *ip = cq->ip;

		qib_update_mmap_info(dev, ip, sz, wc);

		/*
		 * Return the offset to mmap.
		 * See qib_mmap() for details.
		 */
		if (udata && udata->outlen >= sizeof(__u64)) {
			ret = ib_copy_to_udata(udata, &ip->offset,
					       sizeof(ip->offset));
			if (ret)
				goto bail;
		}

		spin_lock_irq(&dev->pending_lock);
		if (list_empty(&ip->pending_mmaps))
			list_add(&ip->pending_mmaps, &dev->pending_mmaps);
		spin_unlock_irq(&dev->pending_lock);
	}

	ret = 0;
	goto bail;

bail_unlock:
	spin_unlock_irq(&cq->lock);
bail_free:
	vfree(wc);
bail:
	return ret;
}
