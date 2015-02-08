

#ifndef _LINUX_SLAB_H
#define	_LINUX_SLAB_H

#include <linux/gfp.h>
#include <linux/types.h>

#define SLAB_DEBUG_FREE		0x00000100UL	/* DEBUG: Perform (expensive) checks on free */
#define SLAB_RED_ZONE		0x00000400UL	/* DEBUG: Red zone objs in a cache */
#define SLAB_POISON		0x00000800UL	/* DEBUG: Poison objects */
#define SLAB_HWCACHE_ALIGN	0x00002000UL	/* Align objs on cache lines */
#define SLAB_CACHE_DMA		0x00004000UL	/* Use GFP_DMA memory */
#define SLAB_STORE_USER		0x00010000UL	/* DEBUG: Store the last owner for bug hunting */
#define SLAB_PANIC		0x00040000UL	/* Panic if kmem_cache_create() fails */
#define SLAB_DESTROY_BY_RCU	0x00080000UL	/* Defer freeing slabs to RCU */
#define SLAB_MEM_SPREAD		0x00100000UL	/* Spread some memory over cpuset */
#define SLAB_TRACE		0x00200000UL	/* Trace allocations and frees */

/* Flag to prevent checks on free */
#ifdef CONFIG_DEBUG_OBJECTS
# define SLAB_DEBUG_OBJECTS	0x00400000UL
#else
# define SLAB_DEBUG_OBJECTS	0x00000000UL
#endif

#define SLAB_NOLEAKTRACE	0x00800000UL	/* Avoid kmemleak tracing */

/* Don't track use of uninitialized memory */
#ifdef CONFIG_KMEMCHECK
# define SLAB_NOTRACK		0x01000000UL
#else
# define SLAB_NOTRACK		0x00000000UL
#endif
#ifdef CONFIG_FAILSLAB
# define SLAB_FAILSLAB		0x02000000UL	/* Fault injection mark */
#else
# define SLAB_FAILSLAB		0x00000000UL
#endif

/* The following flags affect the page allocator grouping pages by mobility */
#define SLAB_RECLAIM_ACCOUNT	0x00020000UL		/* Objects are reclaimable */
#define SLAB_TEMPORARY		SLAB_RECLAIM_ACCOUNT	/* Objects are short-lived */
#define ZERO_SIZE_PTR ((void *)16)

#define ZERO_OR_NULL_PTR(x) ((unsigned long)(x) <= \
				(unsigned long)ZERO_SIZE_PTR)

void __init kmem_cache_init(void);
int slab_is_available(void);

struct kmem_cache *kmem_cache_create(const char *, size_t, size_t,
			unsigned long,
			void (*)(void *));
void kmem_cache_destroy(struct kmem_cache *);
int kmem_cache_shrink(struct kmem_cache *);
void kmem_cache_free(struct kmem_cache *, void *);
unsigned int kmem_cache_size(struct kmem_cache *);
const char *kmem_cache_name(struct kmem_cache *);
int kern_ptr_validate(const void *ptr, unsigned long size);
int kmem_ptr_validate(struct kmem_cache *cachep, const void *ptr);

#define KMEM_CACHE(__struct, __flags) kmem_cache_create(#__struct,\
		sizeof(struct __struct), __alignof__(struct __struct),\
		(__flags), NULL)

#define KMALLOC_SHIFT_HIGH	((MAX_ORDER + PAGE_SHIFT - 1) <= 25 ? \
				(MAX_ORDER + PAGE_SHIFT - 1) : 25)

#define KMALLOC_MAX_SIZE	(1UL << KMALLOC_SHIFT_HIGH)
#define KMALLOC_MAX_ORDER	(KMALLOC_SHIFT_HIGH - PAGE_SHIFT)

void * __must_check __krealloc(const void *, size_t, gfp_t);
void * __must_check krealloc(const void *, size_t, gfp_t);
void kfree(const void *);
void kzfree(const void *);
size_t ksize(const void *);

#ifdef CONFIG_SLUB
#include <linux/slub_def.h>
#elif defined(CONFIG_SLOB)
#include <linux/slob_def.h>
#else
#include <linux/slab_def.h>
#endif

static inline void *kcalloc(size_t n, size_t size, gfp_t flags)
{
	if (size != 0 && n > ULONG_MAX / size)
		return NULL;
	return __kmalloc(n * size, flags | __GFP_ZERO);
}

#if !defined(CONFIG_NUMA) && !defined(CONFIG_SLOB)
static inline void *kmalloc_node(size_t size, gfp_t flags, int node)
{
	return kmalloc(size, flags);
}

static inline void *__kmalloc_node(size_t size, gfp_t flags, int node)
{
	return __kmalloc(size, flags);
}

void *kmem_cache_alloc(struct kmem_cache *, gfp_t);

static inline void *kmem_cache_alloc_node(struct kmem_cache *cachep,
					gfp_t flags, int node)
{
	return kmem_cache_alloc(cachep, flags);
}
#endif /* !CONFIG_NUMA && !CONFIG_SLOB */

#if defined(CONFIG_DEBUG_SLAB) || defined(CONFIG_SLUB)
extern void *__kmalloc_track_caller(size_t, gfp_t, unsigned long);
#define kmalloc_track_caller(size, flags) \
	__kmalloc_track_caller(size, flags, _RET_IP_)
#else
#define kmalloc_track_caller(size, flags) \
	__kmalloc(size, flags)
#endif /* DEBUG_SLAB */

#ifdef CONFIG_NUMA
#if defined(CONFIG_DEBUG_SLAB) || defined(CONFIG_SLUB)
extern void *__kmalloc_node_track_caller(size_t, gfp_t, int, unsigned long);
#define kmalloc_node_track_caller(size, flags, node) \
	__kmalloc_node_track_caller(size, flags, node, \
			_RET_IP_)
#else
#define kmalloc_node_track_caller(size, flags, node) \
	__kmalloc_node(size, flags, node)
#endif

#else /* CONFIG_NUMA */

#define kmalloc_node_track_caller(size, flags, node) \
	kmalloc_track_caller(size, flags)

#endif /* CONFIG_NUMA */

static inline void *kmem_cache_zalloc(struct kmem_cache *k, gfp_t flags)
{
	return kmem_cache_alloc(k, flags | __GFP_ZERO);
}

static inline void *kzalloc(size_t size, gfp_t flags)
{
	return kmalloc(size, flags | __GFP_ZERO);
}

static inline void *kzalloc_node(size_t size, gfp_t flags, int node)
{
	return kmalloc_node(size, flags | __GFP_ZERO, node);
}

void __init kmem_cache_init_late(void);

#endif	/* _LINUX_SLAB_H */
