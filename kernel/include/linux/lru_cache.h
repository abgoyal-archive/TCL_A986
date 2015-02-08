

#ifndef LRU_CACHE_H
#define LRU_CACHE_H

#include <linux/list.h>
#include <linux/slab.h>
#include <linux/bitops.h>
#include <linux/string.h> /* for memset */
#include <linux/seq_file.h>


struct lc_element {
	struct hlist_node colision;
	struct list_head list;		 /* LRU list or free list */
	unsigned refcnt;
	/* back "pointer" into ts_cache->element[index],
	 * for paranoia, and for "ts_element_to_index" */
	unsigned lc_index;
	/* if we want to track a larger set of objects,
	 * it needs to become arch independend u64 */
	unsigned lc_number;

	/* special label when on free list */
#define LC_FREE (~0U)
};

struct lru_cache {
	/* the least recently used item is kept at lru->prev */
	struct list_head lru;
	struct list_head free;
	struct list_head in_use;

	/* the pre-created kmem cache to allocate the objects from */
	struct kmem_cache *lc_cache;

	/* size of tracked objects, used to memset(,0,) them in lc_reset */
	size_t element_size;
	/* offset of struct lc_element member in the tracked object */
	size_t element_off;

	/* number of elements (indices) */
	unsigned int  nr_elements;
	/* Arbitrary limit on maximum tracked objects. Practical limit is much
	 * lower due to allocation failures, probably. For typical use cases,
	 * nr_elements should be a few thousand at most.
	 * This also limits the maximum value of ts_element.ts_index, allowing the
	 * 8 high bits of .ts_index to be overloaded with flags in the future. */
#define LC_MAX_ACTIVE	(1<<24)

	/* statistics */
	unsigned used; /* number of lelements currently on in_use list */
	unsigned long hits, misses, starving, dirty, changed;

	/* see below: flag-bits for lru_cache */
	unsigned long flags;

	/* when changing the label of an index element */
	unsigned int  new_number;

	/* for paranoia when changing the label of an index element */
	struct lc_element *changing_element;

	void  *lc_private;
	const char *name;

	/* nr_elements there */
	struct hlist_head *lc_slot;
	struct lc_element **lc_element;
};


/* flag-bits for lru_cache */
enum {
	/* debugging aid, to catch concurrent access early.
	 * user needs to guarantee exclusive access by proper locking! */
	__LC_PARANOIA,
	/* if we need to change the set, but currently there is a changing
	 * transaction pending, we are "dirty", and must deferr further
	 * changing requests */
	__LC_DIRTY,
	/* if we need to change the set, but currently there is no free nor
	 * unused element available, we are "starving", and must not give out
	 * further references, to guarantee that eventually some refcnt will
	 * drop to zero and we will be able to make progress again, changing
	 * the set, writing the transaction.
	 * if the statistics say we are frequently starving,
	 * nr_elements is too small. */
	__LC_STARVING,
};
#define LC_PARANOIA (1<<__LC_PARANOIA)
#define LC_DIRTY    (1<<__LC_DIRTY)
#define LC_STARVING (1<<__LC_STARVING)

extern struct lru_cache *lc_create(const char *name, struct kmem_cache *cache,
		unsigned e_count, size_t e_size, size_t e_off);
extern void lc_reset(struct lru_cache *lc);
extern void lc_destroy(struct lru_cache *lc);
extern void lc_set(struct lru_cache *lc, unsigned int enr, int index);
extern void lc_del(struct lru_cache *lc, struct lc_element *element);

extern struct lc_element *lc_try_get(struct lru_cache *lc, unsigned int enr);
extern struct lc_element *lc_find(struct lru_cache *lc, unsigned int enr);
extern struct lc_element *lc_get(struct lru_cache *lc, unsigned int enr);
extern unsigned int lc_put(struct lru_cache *lc, struct lc_element *e);
extern void lc_changed(struct lru_cache *lc, struct lc_element *e);

struct seq_file;
extern size_t lc_seq_printf_stats(struct seq_file *seq, struct lru_cache *lc);

extern void lc_seq_dump_details(struct seq_file *seq, struct lru_cache *lc, char *utext,
				void (*detail) (struct seq_file *, struct lc_element *));

static inline int lc_try_lock(struct lru_cache *lc)
{
	return !test_and_set_bit(__LC_DIRTY, &lc->flags);
}

static inline void lc_unlock(struct lru_cache *lc)
{
	clear_bit(__LC_DIRTY, &lc->flags);
	smp_mb__after_clear_bit();
}

static inline int lc_is_used(struct lru_cache *lc, unsigned int enr)
{
	struct lc_element *e = lc_find(lc, enr);
	return e && e->refcnt;
}

#define lc_entry(ptr, type, member) \
	container_of(ptr, type, member)

extern struct lc_element *lc_element_by_index(struct lru_cache *lc, unsigned i);
extern unsigned int lc_index_of(struct lru_cache *lc, struct lc_element *e);

#endif
