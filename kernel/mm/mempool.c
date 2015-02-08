

#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/mempool.h>
#include <linux/blkdev.h>
#include <linux/writeback.h>

static void add_element(mempool_t *pool, void *element)
{
	BUG_ON(pool->curr_nr >= pool->min_nr);
	pool->elements[pool->curr_nr++] = element;
}

static void *remove_element(mempool_t *pool)
{
	BUG_ON(pool->curr_nr <= 0);
	return pool->elements[--pool->curr_nr];
}

static void free_pool(mempool_t *pool)
{
	while (pool->curr_nr) {
		void *element = remove_element(pool);
		pool->free(element, pool->pool_data);
	}
	kfree(pool->elements);
	kfree(pool);
}

mempool_t *mempool_create(int min_nr, mempool_alloc_t *alloc_fn,
				mempool_free_t *free_fn, void *pool_data)
{
	return  mempool_create_node(min_nr,alloc_fn,free_fn, pool_data,-1);
}
EXPORT_SYMBOL(mempool_create);

mempool_t *mempool_create_node(int min_nr, mempool_alloc_t *alloc_fn,
			mempool_free_t *free_fn, void *pool_data, int node_id)
{
	mempool_t *pool;
	pool = kmalloc_node(sizeof(*pool), GFP_KERNEL | __GFP_ZERO, node_id);
	if (!pool)
		return NULL;
	pool->elements = kmalloc_node(min_nr * sizeof(void *),
					GFP_KERNEL, node_id);
	if (!pool->elements) {
		kfree(pool);
		return NULL;
	}
	spin_lock_init(&pool->lock);
	pool->min_nr = min_nr;
	pool->pool_data = pool_data;
	init_waitqueue_head(&pool->wait);
	pool->alloc = alloc_fn;
	pool->free = free_fn;

	/*
	 * First pre-allocate the guaranteed number of buffers.
	 */
	while (pool->curr_nr < pool->min_nr) {
		void *element;

		element = pool->alloc(GFP_KERNEL, pool->pool_data);
		if (unlikely(!element)) {
			free_pool(pool);
			return NULL;
		}
		add_element(pool, element);
	}
	return pool;
}
EXPORT_SYMBOL(mempool_create_node);

int mempool_resize(mempool_t *pool, int new_min_nr, gfp_t gfp_mask)
{
	void *element;
	void **new_elements;
	unsigned long flags;

	BUG_ON(new_min_nr <= 0);

	spin_lock_irqsave(&pool->lock, flags);
	if (new_min_nr <= pool->min_nr) {
		while (new_min_nr < pool->curr_nr) {
			element = remove_element(pool);
			spin_unlock_irqrestore(&pool->lock, flags);
			pool->free(element, pool->pool_data);
			spin_lock_irqsave(&pool->lock, flags);
		}
		pool->min_nr = new_min_nr;
		goto out_unlock;
	}
	spin_unlock_irqrestore(&pool->lock, flags);

	/* Grow the pool */
	new_elements = kmalloc(new_min_nr * sizeof(*new_elements), gfp_mask);
	if (!new_elements)
		return -ENOMEM;

	spin_lock_irqsave(&pool->lock, flags);
	if (unlikely(new_min_nr <= pool->min_nr)) {
		/* Raced, other resize will do our work */
		spin_unlock_irqrestore(&pool->lock, flags);
		kfree(new_elements);
		goto out;
	}
	memcpy(new_elements, pool->elements,
			pool->curr_nr * sizeof(*new_elements));
	kfree(pool->elements);
	pool->elements = new_elements;
	pool->min_nr = new_min_nr;

	while (pool->curr_nr < pool->min_nr) {
		spin_unlock_irqrestore(&pool->lock, flags);
		element = pool->alloc(gfp_mask, pool->pool_data);
		if (!element)
			goto out;
		spin_lock_irqsave(&pool->lock, flags);
		if (pool->curr_nr < pool->min_nr) {
			add_element(pool, element);
		} else {
			spin_unlock_irqrestore(&pool->lock, flags);
			pool->free(element, pool->pool_data);	/* Raced */
			goto out;
		}
	}
out_unlock:
	spin_unlock_irqrestore(&pool->lock, flags);
out:
	return 0;
}
EXPORT_SYMBOL(mempool_resize);

void mempool_destroy(mempool_t *pool)
{
	/* Check for outstanding elements */
	BUG_ON(pool->curr_nr != pool->min_nr);
	free_pool(pool);
}
EXPORT_SYMBOL(mempool_destroy);

void * mempool_alloc(mempool_t *pool, gfp_t gfp_mask)
{
	void *element;
	unsigned long flags;
	wait_queue_t wait;
	gfp_t gfp_temp;

	might_sleep_if(gfp_mask & __GFP_WAIT);

	gfp_mask |= __GFP_NOMEMALLOC;	/* don't allocate emergency reserves */
	gfp_mask |= __GFP_NORETRY;	/* don't loop in __alloc_pages */
	gfp_mask |= __GFP_NOWARN;	/* failures are OK */

	gfp_temp = gfp_mask & ~(__GFP_WAIT|__GFP_IO);

repeat_alloc:

	element = pool->alloc(gfp_temp, pool->pool_data);
	if (likely(element != NULL))
		return element;

	spin_lock_irqsave(&pool->lock, flags);
	if (likely(pool->curr_nr)) {
		element = remove_element(pool);
		spin_unlock_irqrestore(&pool->lock, flags);
		return element;
	}
	spin_unlock_irqrestore(&pool->lock, flags);

	/* We must not sleep in the GFP_ATOMIC case */
	if (!(gfp_mask & __GFP_WAIT))
		return NULL;

	/* Now start performing page reclaim */
	gfp_temp = gfp_mask;
	init_wait(&wait);
	prepare_to_wait(&pool->wait, &wait, TASK_UNINTERRUPTIBLE);
	smp_mb();
	if (!pool->curr_nr) {
		/*
		 * FIXME: this should be io_schedule().  The timeout is there
		 * as a workaround for some DM problems in 2.6.18.
		 */
		io_schedule_timeout(5*HZ);
	}
	finish_wait(&pool->wait, &wait);

	goto repeat_alloc;
}
EXPORT_SYMBOL(mempool_alloc);

void mempool_free(void *element, mempool_t *pool)
{
	unsigned long flags;

	if (unlikely(element == NULL))
		return;

	smp_mb();
	if (pool->curr_nr < pool->min_nr) {
		spin_lock_irqsave(&pool->lock, flags);
		if (pool->curr_nr < pool->min_nr) {
			add_element(pool, element);
			spin_unlock_irqrestore(&pool->lock, flags);
			wake_up(&pool->wait);
			return;
		}
		spin_unlock_irqrestore(&pool->lock, flags);
	}
	pool->free(element, pool->pool_data);
}
EXPORT_SYMBOL(mempool_free);

void *mempool_alloc_slab(gfp_t gfp_mask, void *pool_data)
{
	struct kmem_cache *mem = pool_data;
	return kmem_cache_alloc(mem, gfp_mask);
}
EXPORT_SYMBOL(mempool_alloc_slab);

void mempool_free_slab(void *element, void *pool_data)
{
	struct kmem_cache *mem = pool_data;
	kmem_cache_free(mem, element);
}
EXPORT_SYMBOL(mempool_free_slab);

void *mempool_kmalloc(gfp_t gfp_mask, void *pool_data)
{
	size_t size = (size_t)pool_data;
	return kmalloc(size, gfp_mask);
}
EXPORT_SYMBOL(mempool_kmalloc);

void mempool_kfree(void *element, void *pool_data)
{
	kfree(element);
}
EXPORT_SYMBOL(mempool_kfree);

void *mempool_alloc_pages(gfp_t gfp_mask, void *pool_data)
{
	int order = (int)(long)pool_data;
	return alloc_pages(gfp_mask, order);
}
EXPORT_SYMBOL(mempool_alloc_pages);

void mempool_free_pages(void *element, void *pool_data)
{
	int order = (int)(long)pool_data;
	__free_pages(element, order);
}
EXPORT_SYMBOL(mempool_free_pages);
