#ifndef DMAENGINE_H
#define DMAENGINE_H

#ifdef CONFIG_DMA_ENGINE

#include <linux/device.h>
#include <linux/uio.h>
#include <linux/kref.h>
#include <linux/completion.h>
#include <linux/rcupdate.h>

enum dma_event {
	DMA_RESOURCE_SUSPEND,
	DMA_RESOURCE_RESUME,
	DMA_RESOURCE_ADDED,
	DMA_RESOURCE_REMOVED,
};

typedef s32 dma_cookie_t;

#define dma_submit_error(cookie) ((cookie) < 0 ? 1 : 0)

enum dma_status {
	DMA_SUCCESS,
	DMA_IN_PROGRESS,
	DMA_ERROR,
};


struct dma_chan_percpu {
	local_t refcount;
	/* stats */
	unsigned long memcpy_count;
	unsigned long bytes_transferred;
};

struct dma_chan {
	struct dma_client *client;
	struct dma_device *device;
	dma_cookie_t cookie;

	/* sysfs */
	int chan_id;
	struct class_device class_dev;

	struct kref refcount;
	int slow_ref;
	struct rcu_head rcu;

	struct list_head client_node;
	struct list_head device_node;
	struct dma_chan_percpu *local;
};

void dma_chan_cleanup(struct kref *kref);

static inline void dma_chan_get(struct dma_chan *chan)
{
	if (unlikely(chan->slow_ref))
		kref_get(&chan->refcount);
	else {
		local_inc(&(per_cpu_ptr(chan->local, get_cpu())->refcount));
		put_cpu();
	}
}

static inline void dma_chan_put(struct dma_chan *chan)
{
	if (unlikely(chan->slow_ref))
		kref_put(&chan->refcount, dma_chan_cleanup);
	else {
		local_dec(&(per_cpu_ptr(chan->local, get_cpu())->refcount));
		put_cpu();
	}
}

typedef void (*dma_event_callback) (struct dma_client *client,
		struct dma_chan *chan, enum dma_event event);

struct dma_client {
	dma_event_callback	event_callback;
	unsigned int		chan_count;
	unsigned int		chans_desired;

	spinlock_t		lock;
	struct list_head	channels;
	struct list_head	global_node;
};

struct dma_device {

	unsigned int chancnt;
	struct list_head channels;
	struct list_head global_node;

	struct kref refcount;
	struct completion done;

	int dev_id;

	int (*device_alloc_chan_resources)(struct dma_chan *chan);
	void (*device_free_chan_resources)(struct dma_chan *chan);
	dma_cookie_t (*device_memcpy_buf_to_buf)(struct dma_chan *chan,
			void *dest, void *src, size_t len);
	dma_cookie_t (*device_memcpy_buf_to_pg)(struct dma_chan *chan,
			struct page *page, unsigned int offset, void *kdata,
			size_t len);
	dma_cookie_t (*device_memcpy_pg_to_pg)(struct dma_chan *chan,
			struct page *dest_pg, unsigned int dest_off,
			struct page *src_pg, unsigned int src_off, size_t len);
	enum dma_status (*device_memcpy_complete)(struct dma_chan *chan,
			dma_cookie_t cookie, dma_cookie_t *last,
			dma_cookie_t *used);
	void (*device_memcpy_issue_pending)(struct dma_chan *chan);
};

/* --- public DMA engine API --- */

struct dma_client *dma_async_client_register(dma_event_callback event_callback);
void dma_async_client_unregister(struct dma_client *client);
void dma_async_client_chan_request(struct dma_client *client,
		unsigned int number);

static inline dma_cookie_t dma_async_memcpy_buf_to_buf(struct dma_chan *chan,
	void *dest, void *src, size_t len)
{
	int cpu = get_cpu();
	per_cpu_ptr(chan->local, cpu)->bytes_transferred += len;
	per_cpu_ptr(chan->local, cpu)->memcpy_count++;
	put_cpu();

	return chan->device->device_memcpy_buf_to_buf(chan, dest, src, len);
}

static inline dma_cookie_t dma_async_memcpy_buf_to_pg(struct dma_chan *chan,
	struct page *page, unsigned int offset, void *kdata, size_t len)
{
	int cpu = get_cpu();
	per_cpu_ptr(chan->local, cpu)->bytes_transferred += len;
	per_cpu_ptr(chan->local, cpu)->memcpy_count++;
	put_cpu();

	return chan->device->device_memcpy_buf_to_pg(chan, page, offset,
	                                             kdata, len);
}

static inline dma_cookie_t dma_async_memcpy_pg_to_pg(struct dma_chan *chan,
	struct page *dest_pg, unsigned int dest_off, struct page *src_pg,
	unsigned int src_off, size_t len)
{
	int cpu = get_cpu();
	per_cpu_ptr(chan->local, cpu)->bytes_transferred += len;
	per_cpu_ptr(chan->local, cpu)->memcpy_count++;
	put_cpu();

	return chan->device->device_memcpy_pg_to_pg(chan, dest_pg, dest_off,
	                                            src_pg, src_off, len);
}

static inline void dma_async_memcpy_issue_pending(struct dma_chan *chan)
{
	return chan->device->device_memcpy_issue_pending(chan);
}

static inline enum dma_status dma_async_memcpy_complete(struct dma_chan *chan,
	dma_cookie_t cookie, dma_cookie_t *last, dma_cookie_t *used)
{
	return chan->device->device_memcpy_complete(chan, cookie, last, used);
}

static inline enum dma_status dma_async_is_complete(dma_cookie_t cookie,
			dma_cookie_t last_complete, dma_cookie_t last_used)
{
	if (last_complete <= last_used) {
		if ((cookie <= last_complete) || (cookie > last_used))
			return DMA_SUCCESS;
	} else {
		if ((cookie <= last_complete) && (cookie > last_used))
			return DMA_SUCCESS;
	}
	return DMA_IN_PROGRESS;
}


/* --- DMA device --- */

int dma_async_device_register(struct dma_device *device);
void dma_async_device_unregister(struct dma_device *device);

/* --- Helper iov-locking functions --- */

struct dma_page_list {
	char *base_address;
	int nr_pages;
	struct page **pages;
};

struct dma_pinned_list {
	int nr_iovecs;
	struct dma_page_list page_list[0];
};

struct dma_pinned_list *dma_pin_iovec_pages(struct iovec *iov, size_t len);
void dma_unpin_iovec_pages(struct dma_pinned_list* pinned_list);

dma_cookie_t dma_memcpy_to_iovec(struct dma_chan *chan, struct iovec *iov,
	struct dma_pinned_list *pinned_list, unsigned char *kdata, size_t len);
dma_cookie_t dma_memcpy_pg_to_iovec(struct dma_chan *chan, struct iovec *iov,
	struct dma_pinned_list *pinned_list, struct page *page,
	unsigned int offset, size_t len);

#endif /* CONFIG_DMA_ENGINE */
#endif /* DMAENGINE_H */
