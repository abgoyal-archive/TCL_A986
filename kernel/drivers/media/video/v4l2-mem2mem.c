
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/slab.h>

#include <media/videobuf-core.h>
#include <media/v4l2-mem2mem.h>

MODULE_DESCRIPTION("Mem to mem device framework for videobuf");
MODULE_AUTHOR("Pawel Osciak, <p.osciak@samsung.com>");
MODULE_LICENSE("GPL");

static bool debug;
module_param(debug, bool, 0644);

#define dprintk(fmt, arg...)						\
	do {								\
		if (debug)						\
			printk(KERN_DEBUG "%s: " fmt, __func__, ## arg);\
	} while (0)


/* Instance is already queued on the job_queue */
#define TRANS_QUEUED		(1 << 0)
/* Instance is currently running in hardware */
#define TRANS_RUNNING		(1 << 1)


#define DST_QUEUE_OFF_BASE	(1 << 30)


struct v4l2_m2m_dev {
	struct v4l2_m2m_ctx	*curr_ctx;

	struct list_head	job_queue;
	spinlock_t		job_spinlock;

	struct v4l2_m2m_ops	*m2m_ops;
};

static struct v4l2_m2m_queue_ctx *get_queue_ctx(struct v4l2_m2m_ctx *m2m_ctx,
						enum v4l2_buf_type type)
{
	switch (type) {
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		return &m2m_ctx->cap_q_ctx;
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
		return &m2m_ctx->out_q_ctx;
	default:
		printk(KERN_ERR "Invalid buffer type\n");
		return NULL;
	}
}

struct videobuf_queue *v4l2_m2m_get_vq(struct v4l2_m2m_ctx *m2m_ctx,
				       enum v4l2_buf_type type)
{
	struct v4l2_m2m_queue_ctx *q_ctx;

	q_ctx = get_queue_ctx(m2m_ctx, type);
	if (!q_ctx)
		return NULL;

	return &q_ctx->q;
}
EXPORT_SYMBOL(v4l2_m2m_get_vq);

void *v4l2_m2m_next_buf(struct v4l2_m2m_ctx *m2m_ctx, enum v4l2_buf_type type)
{
	struct v4l2_m2m_queue_ctx *q_ctx;
	struct videobuf_buffer *vb = NULL;
	unsigned long flags;

	q_ctx = get_queue_ctx(m2m_ctx, type);
	if (!q_ctx)
		return NULL;

	spin_lock_irqsave(q_ctx->q.irqlock, flags);

	if (list_empty(&q_ctx->rdy_queue))
		goto end;

	vb = list_entry(q_ctx->rdy_queue.next, struct videobuf_buffer, queue);
	vb->state = VIDEOBUF_ACTIVE;

end:
	spin_unlock_irqrestore(q_ctx->q.irqlock, flags);
	return vb;
}
EXPORT_SYMBOL_GPL(v4l2_m2m_next_buf);

void *v4l2_m2m_buf_remove(struct v4l2_m2m_ctx *m2m_ctx, enum v4l2_buf_type type)
{
	struct v4l2_m2m_queue_ctx *q_ctx;
	struct videobuf_buffer *vb = NULL;
	unsigned long flags;

	q_ctx = get_queue_ctx(m2m_ctx, type);
	if (!q_ctx)
		return NULL;

	spin_lock_irqsave(q_ctx->q.irqlock, flags);
	if (!list_empty(&q_ctx->rdy_queue)) {
		vb = list_entry(q_ctx->rdy_queue.next, struct videobuf_buffer,
				queue);
		list_del(&vb->queue);
		q_ctx->num_rdy--;
	}
	spin_unlock_irqrestore(q_ctx->q.irqlock, flags);

	return vb;
}
EXPORT_SYMBOL_GPL(v4l2_m2m_buf_remove);


void *v4l2_m2m_get_curr_priv(struct v4l2_m2m_dev *m2m_dev)
{
	unsigned long flags;
	void *ret = NULL;

	spin_lock_irqsave(&m2m_dev->job_spinlock, flags);
	if (m2m_dev->curr_ctx)
		ret = m2m_dev->curr_ctx->priv;
	spin_unlock_irqrestore(&m2m_dev->job_spinlock, flags);

	return ret;
}
EXPORT_SYMBOL(v4l2_m2m_get_curr_priv);

static void v4l2_m2m_try_run(struct v4l2_m2m_dev *m2m_dev)
{
	unsigned long flags;

	spin_lock_irqsave(&m2m_dev->job_spinlock, flags);
	if (NULL != m2m_dev->curr_ctx) {
		spin_unlock_irqrestore(&m2m_dev->job_spinlock, flags);
		dprintk("Another instance is running, won't run now\n");
		return;
	}

	if (list_empty(&m2m_dev->job_queue)) {
		spin_unlock_irqrestore(&m2m_dev->job_spinlock, flags);
		dprintk("No job pending\n");
		return;
	}

	m2m_dev->curr_ctx = list_entry(m2m_dev->job_queue.next,
				   struct v4l2_m2m_ctx, queue);
	m2m_dev->curr_ctx->job_flags |= TRANS_RUNNING;
	spin_unlock_irqrestore(&m2m_dev->job_spinlock, flags);

	m2m_dev->m2m_ops->device_run(m2m_dev->curr_ctx->priv);
}

static void v4l2_m2m_try_schedule(struct v4l2_m2m_ctx *m2m_ctx)
{
	struct v4l2_m2m_dev *m2m_dev;
	unsigned long flags_job, flags;

	m2m_dev = m2m_ctx->m2m_dev;
	dprintk("Trying to schedule a job for m2m_ctx: %p\n", m2m_ctx);

	if (!m2m_ctx->out_q_ctx.q.streaming
	    || !m2m_ctx->cap_q_ctx.q.streaming) {
		dprintk("Streaming needs to be on for both queues\n");
		return;
	}

	spin_lock_irqsave(&m2m_dev->job_spinlock, flags_job);
	if (m2m_ctx->job_flags & TRANS_QUEUED) {
		spin_unlock_irqrestore(&m2m_dev->job_spinlock, flags_job);
		dprintk("On job queue already\n");
		return;
	}

	spin_lock_irqsave(m2m_ctx->out_q_ctx.q.irqlock, flags);
	if (list_empty(&m2m_ctx->out_q_ctx.rdy_queue)) {
		spin_unlock_irqrestore(m2m_ctx->out_q_ctx.q.irqlock, flags);
		spin_unlock_irqrestore(&m2m_dev->job_spinlock, flags_job);
		dprintk("No input buffers available\n");
		return;
	}
	if (list_empty(&m2m_ctx->cap_q_ctx.rdy_queue)) {
		spin_unlock_irqrestore(m2m_ctx->out_q_ctx.q.irqlock, flags);
		spin_unlock_irqrestore(&m2m_dev->job_spinlock, flags_job);
		dprintk("No output buffers available\n");
		return;
	}
	spin_unlock_irqrestore(m2m_ctx->out_q_ctx.q.irqlock, flags);

	if (m2m_dev->m2m_ops->job_ready
		&& (!m2m_dev->m2m_ops->job_ready(m2m_ctx->priv))) {
		spin_unlock_irqrestore(&m2m_dev->job_spinlock, flags_job);
		dprintk("Driver not ready\n");
		return;
	}

	list_add_tail(&m2m_ctx->queue, &m2m_dev->job_queue);
	m2m_ctx->job_flags |= TRANS_QUEUED;

	spin_unlock_irqrestore(&m2m_dev->job_spinlock, flags_job);

	v4l2_m2m_try_run(m2m_dev);
}

void v4l2_m2m_job_finish(struct v4l2_m2m_dev *m2m_dev,
			 struct v4l2_m2m_ctx *m2m_ctx)
{
	unsigned long flags;

	spin_lock_irqsave(&m2m_dev->job_spinlock, flags);
	if (!m2m_dev->curr_ctx || m2m_dev->curr_ctx != m2m_ctx) {
		spin_unlock_irqrestore(&m2m_dev->job_spinlock, flags);
		dprintk("Called by an instance not currently running\n");
		return;
	}

	list_del(&m2m_dev->curr_ctx->queue);
	m2m_dev->curr_ctx->job_flags &= ~(TRANS_QUEUED | TRANS_RUNNING);
	m2m_dev->curr_ctx = NULL;

	spin_unlock_irqrestore(&m2m_dev->job_spinlock, flags);

	/* This instance might have more buffers ready, but since we do not
	 * allow more than one job on the job_queue per instance, each has
	 * to be scheduled separately after the previous one finishes. */
	v4l2_m2m_try_schedule(m2m_ctx);
	v4l2_m2m_try_run(m2m_dev);
}
EXPORT_SYMBOL(v4l2_m2m_job_finish);

int v4l2_m2m_reqbufs(struct file *file, struct v4l2_m2m_ctx *m2m_ctx,
		     struct v4l2_requestbuffers *reqbufs)
{
	struct videobuf_queue *vq;

	vq = v4l2_m2m_get_vq(m2m_ctx, reqbufs->type);
	return videobuf_reqbufs(vq, reqbufs);
}
EXPORT_SYMBOL_GPL(v4l2_m2m_reqbufs);

int v4l2_m2m_querybuf(struct file *file, struct v4l2_m2m_ctx *m2m_ctx,
		      struct v4l2_buffer *buf)
{
	struct videobuf_queue *vq;
	int ret;

	vq = v4l2_m2m_get_vq(m2m_ctx, buf->type);
	ret = videobuf_querybuf(vq, buf);

	if (buf->memory == V4L2_MEMORY_MMAP
	    && vq->type == V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		buf->m.offset += DST_QUEUE_OFF_BASE;
	}

	return ret;
}
EXPORT_SYMBOL_GPL(v4l2_m2m_querybuf);

int v4l2_m2m_qbuf(struct file *file, struct v4l2_m2m_ctx *m2m_ctx,
		  struct v4l2_buffer *buf)
{
	struct videobuf_queue *vq;
	int ret;

	vq = v4l2_m2m_get_vq(m2m_ctx, buf->type);
	ret = videobuf_qbuf(vq, buf);
	if (!ret)
		v4l2_m2m_try_schedule(m2m_ctx);

	return ret;
}
EXPORT_SYMBOL_GPL(v4l2_m2m_qbuf);

int v4l2_m2m_dqbuf(struct file *file, struct v4l2_m2m_ctx *m2m_ctx,
		   struct v4l2_buffer *buf)
{
	struct videobuf_queue *vq;

	vq = v4l2_m2m_get_vq(m2m_ctx, buf->type);
	return videobuf_dqbuf(vq, buf, file->f_flags & O_NONBLOCK);
}
EXPORT_SYMBOL_GPL(v4l2_m2m_dqbuf);

int v4l2_m2m_streamon(struct file *file, struct v4l2_m2m_ctx *m2m_ctx,
		      enum v4l2_buf_type type)
{
	struct videobuf_queue *vq;
	int ret;

	vq = v4l2_m2m_get_vq(m2m_ctx, type);
	ret = videobuf_streamon(vq);
	if (!ret)
		v4l2_m2m_try_schedule(m2m_ctx);

	return ret;
}
EXPORT_SYMBOL_GPL(v4l2_m2m_streamon);

int v4l2_m2m_streamoff(struct file *file, struct v4l2_m2m_ctx *m2m_ctx,
		       enum v4l2_buf_type type)
{
	struct videobuf_queue *vq;

	vq = v4l2_m2m_get_vq(m2m_ctx, type);
	return videobuf_streamoff(vq);
}
EXPORT_SYMBOL_GPL(v4l2_m2m_streamoff);

unsigned int v4l2_m2m_poll(struct file *file, struct v4l2_m2m_ctx *m2m_ctx,
			   struct poll_table_struct *wait)
{
	struct videobuf_queue *src_q, *dst_q;
	struct videobuf_buffer *src_vb = NULL, *dst_vb = NULL;
	unsigned int rc = 0;

	src_q = v4l2_m2m_get_src_vq(m2m_ctx);
	dst_q = v4l2_m2m_get_dst_vq(m2m_ctx);

	mutex_lock(&src_q->vb_lock);
	mutex_lock(&dst_q->vb_lock);

	if (src_q->streaming && !list_empty(&src_q->stream))
		src_vb = list_first_entry(&src_q->stream,
					  struct videobuf_buffer, stream);
	if (dst_q->streaming && !list_empty(&dst_q->stream))
		dst_vb = list_first_entry(&dst_q->stream,
					  struct videobuf_buffer, stream);

	if (!src_vb && !dst_vb) {
		rc = POLLERR;
		goto end;
	}

	if (src_vb) {
		poll_wait(file, &src_vb->done, wait);
		if (src_vb->state == VIDEOBUF_DONE
		    || src_vb->state == VIDEOBUF_ERROR)
			rc |= POLLOUT | POLLWRNORM;
	}
	if (dst_vb) {
		poll_wait(file, &dst_vb->done, wait);
		if (dst_vb->state == VIDEOBUF_DONE
		    || dst_vb->state == VIDEOBUF_ERROR)
			rc |= POLLIN | POLLRDNORM;
	}

end:
	mutex_unlock(&dst_q->vb_lock);
	mutex_unlock(&src_q->vb_lock);
	return rc;
}
EXPORT_SYMBOL_GPL(v4l2_m2m_poll);

int v4l2_m2m_mmap(struct file *file, struct v4l2_m2m_ctx *m2m_ctx,
			 struct vm_area_struct *vma)
{
	unsigned long offset = vma->vm_pgoff << PAGE_SHIFT;
	struct videobuf_queue *vq;

	if (offset < DST_QUEUE_OFF_BASE) {
		vq = v4l2_m2m_get_src_vq(m2m_ctx);
	} else {
		vq = v4l2_m2m_get_dst_vq(m2m_ctx);
		vma->vm_pgoff -= (DST_QUEUE_OFF_BASE >> PAGE_SHIFT);
	}

	return videobuf_mmap_mapper(vq, vma);
}
EXPORT_SYMBOL(v4l2_m2m_mmap);

struct v4l2_m2m_dev *v4l2_m2m_init(struct v4l2_m2m_ops *m2m_ops)
{
	struct v4l2_m2m_dev *m2m_dev;

	if (!m2m_ops)
		return ERR_PTR(-EINVAL);

	BUG_ON(!m2m_ops->device_run);
	BUG_ON(!m2m_ops->job_abort);

	m2m_dev = kzalloc(sizeof *m2m_dev, GFP_KERNEL);
	if (!m2m_dev)
		return ERR_PTR(-ENOMEM);

	m2m_dev->curr_ctx = NULL;
	m2m_dev->m2m_ops = m2m_ops;
	INIT_LIST_HEAD(&m2m_dev->job_queue);
	spin_lock_init(&m2m_dev->job_spinlock);

	return m2m_dev;
}
EXPORT_SYMBOL_GPL(v4l2_m2m_init);

void v4l2_m2m_release(struct v4l2_m2m_dev *m2m_dev)
{
	kfree(m2m_dev);
}
EXPORT_SYMBOL_GPL(v4l2_m2m_release);

struct v4l2_m2m_ctx *v4l2_m2m_ctx_init(void *priv, struct v4l2_m2m_dev *m2m_dev,
			void (*vq_init)(void *priv, struct videobuf_queue *,
					enum v4l2_buf_type))
{
	struct v4l2_m2m_ctx *m2m_ctx;
	struct v4l2_m2m_queue_ctx *out_q_ctx, *cap_q_ctx;

	if (!vq_init)
		return ERR_PTR(-EINVAL);

	m2m_ctx = kzalloc(sizeof *m2m_ctx, GFP_KERNEL);
	if (!m2m_ctx)
		return ERR_PTR(-ENOMEM);

	m2m_ctx->priv = priv;
	m2m_ctx->m2m_dev = m2m_dev;

	out_q_ctx = get_queue_ctx(m2m_ctx, V4L2_BUF_TYPE_VIDEO_OUTPUT);
	cap_q_ctx = get_queue_ctx(m2m_ctx, V4L2_BUF_TYPE_VIDEO_CAPTURE);

	INIT_LIST_HEAD(&out_q_ctx->rdy_queue);
	INIT_LIST_HEAD(&cap_q_ctx->rdy_queue);

	INIT_LIST_HEAD(&m2m_ctx->queue);

	vq_init(priv, &out_q_ctx->q, V4L2_BUF_TYPE_VIDEO_OUTPUT);
	vq_init(priv, &cap_q_ctx->q, V4L2_BUF_TYPE_VIDEO_CAPTURE);
	out_q_ctx->q.priv_data = cap_q_ctx->q.priv_data = priv;

	return m2m_ctx;
}
EXPORT_SYMBOL_GPL(v4l2_m2m_ctx_init);

void v4l2_m2m_ctx_release(struct v4l2_m2m_ctx *m2m_ctx)
{
	struct v4l2_m2m_dev *m2m_dev;
	struct videobuf_buffer *vb;
	unsigned long flags;

	m2m_dev = m2m_ctx->m2m_dev;

	spin_lock_irqsave(&m2m_dev->job_spinlock, flags);
	if (m2m_ctx->job_flags & TRANS_RUNNING) {
		spin_unlock_irqrestore(&m2m_dev->job_spinlock, flags);
		m2m_dev->m2m_ops->job_abort(m2m_ctx->priv);
		dprintk("m2m_ctx %p running, will wait to complete", m2m_ctx);
		vb = v4l2_m2m_next_dst_buf(m2m_ctx);
		BUG_ON(NULL == vb);
		wait_event(vb->done, vb->state != VIDEOBUF_ACTIVE
				     && vb->state != VIDEOBUF_QUEUED);
	} else if (m2m_ctx->job_flags & TRANS_QUEUED) {
		list_del(&m2m_ctx->queue);
		m2m_ctx->job_flags &= ~(TRANS_QUEUED | TRANS_RUNNING);
		spin_unlock_irqrestore(&m2m_dev->job_spinlock, flags);
		dprintk("m2m_ctx: %p had been on queue and was removed\n",
			m2m_ctx);
	} else {
		/* Do nothing, was not on queue/running */
		spin_unlock_irqrestore(&m2m_dev->job_spinlock, flags);
	}

	videobuf_stop(&m2m_ctx->cap_q_ctx.q);
	videobuf_stop(&m2m_ctx->out_q_ctx.q);

	videobuf_mmap_free(&m2m_ctx->cap_q_ctx.q);
	videobuf_mmap_free(&m2m_ctx->out_q_ctx.q);

	kfree(m2m_ctx);
}
EXPORT_SYMBOL_GPL(v4l2_m2m_ctx_release);

void v4l2_m2m_buf_queue(struct v4l2_m2m_ctx *m2m_ctx, struct videobuf_queue *vq,
			struct videobuf_buffer *vb)
{
	struct v4l2_m2m_queue_ctx *q_ctx;

	q_ctx = get_queue_ctx(m2m_ctx, vq->type);
	if (!q_ctx)
		return;

	list_add_tail(&vb->queue, &q_ctx->rdy_queue);
	q_ctx->num_rdy++;

	vb->state = VIDEOBUF_QUEUED;
}
EXPORT_SYMBOL_GPL(v4l2_m2m_buf_queue);

