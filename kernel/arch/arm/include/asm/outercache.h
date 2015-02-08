

#ifndef __ASM_OUTERCACHE_H
#define __ASM_OUTERCACHE_H

struct outer_cache_fns {
	void (*inv_range)(unsigned long, unsigned long);
	void (*clean_range)(unsigned long, unsigned long);
	void (*flush_range)(unsigned long, unsigned long);
        void (*clean_all)(void);
	void (*flush_all)(void);
#ifdef CONFIG_OUTER_CACHE_SYNC
	void (*sync)(void);
#endif
};

#ifdef CONFIG_OUTER_CACHE

extern struct outer_cache_fns outer_cache;

static inline void outer_inv_range(unsigned long start, unsigned long end)
{
	if (outer_cache.inv_range)
		outer_cache.inv_range(start, end);
}
static inline void outer_clean_range(unsigned long start, unsigned long end)
{
	if (outer_cache.clean_range)
		outer_cache.clean_range(start, end);
}
static inline void outer_flush_range(unsigned long start, unsigned long end)
{
	if (outer_cache.flush_range)
		outer_cache.flush_range(start, end);
}
static inline void outer_clean_all(void)
{
	if (outer_cache.clean_all)
		outer_cache.clean_all();
}
static inline void outer_flush_all(void)
{
	if (outer_cache.flush_all)
		outer_cache.flush_all();
}

#else

static inline void outer_inv_range(unsigned long start, unsigned long end)
{ }
static inline void outer_clean_range(unsigned long start, unsigned long end)
{ }
static inline void outer_flush_range(unsigned long start, unsigned long end)
{ }
static inline void outer_clean_all(void)
{ }
static inline void outer_flush_all(void)
{ }

#endif

#ifdef CONFIG_OUTER_CACHE_SYNC
static inline void outer_sync(void)
{
	if (outer_cache.sync)
		outer_cache.sync();
}
#else
static inline void outer_sync(void)
{ }
#endif

#endif	/* __ASM_OUTERCACHE_H */
