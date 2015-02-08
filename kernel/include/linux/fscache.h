

#ifndef _LINUX_FSCACHE_H
#define _LINUX_FSCACHE_H

#include <linux/fs.h>
#include <linux/list.h>
#include <linux/pagemap.h>
#include <linux/pagevec.h>

#if defined(CONFIG_FSCACHE) || defined(CONFIG_FSCACHE_MODULE)
#define fscache_available() (1)
#define fscache_cookie_valid(cookie) (cookie)
#else
#define fscache_available() (0)
#define fscache_cookie_valid(cookie) (0)
#endif


#define PageFsCache(page)		PagePrivate2((page))
#define SetPageFsCache(page)		SetPagePrivate2((page))
#define ClearPageFsCache(page)		ClearPagePrivate2((page))
#define TestSetPageFsCache(page)	TestSetPagePrivate2((page))
#define TestClearPageFsCache(page)	TestClearPagePrivate2((page))

/* pattern used to fill dead space in an index entry */
#define FSCACHE_INDEX_DEADFILL_PATTERN 0x79

struct pagevec;
struct fscache_cache_tag;
struct fscache_cookie;
struct fscache_netfs;

typedef void (*fscache_rw_complete_t)(struct page *page,
				      void *context,
				      int error);

/* result of index entry consultation */
enum fscache_checkaux {
	FSCACHE_CHECKAUX_OKAY,		/* entry okay as is */
	FSCACHE_CHECKAUX_NEEDS_UPDATE,	/* entry requires update */
	FSCACHE_CHECKAUX_OBSOLETE,	/* entry requires deletion */
};

struct fscache_cookie_def {
	/* name of cookie type */
	char name[16];

	/* cookie type */
	uint8_t type;
#define FSCACHE_COOKIE_TYPE_INDEX	0
#define FSCACHE_COOKIE_TYPE_DATAFILE	1

	/* select the cache into which to insert an entry in this index
	 * - optional
	 * - should return a cache identifier or NULL to cause the cache to be
	 *   inherited from the parent if possible or the first cache picked
	 *   for a non-index file if not
	 */
	struct fscache_cache_tag *(*select_cache)(
		const void *parent_netfs_data,
		const void *cookie_netfs_data);

	/* get an index key
	 * - should store the key data in the buffer
	 * - should return the amount of amount stored
	 * - not permitted to return an error
	 * - the netfs data from the cookie being used as the source is
	 *   presented
	 */
	uint16_t (*get_key)(const void *cookie_netfs_data,
			    void *buffer,
			    uint16_t bufmax);

	/* get certain file attributes from the netfs data
	 * - this function can be absent for an index
	 * - not permitted to return an error
	 * - the netfs data from the cookie being used as the source is
	 *   presented
	 */
	void (*get_attr)(const void *cookie_netfs_data, uint64_t *size);

	/* get the auxilliary data from netfs data
	 * - this function can be absent if the index carries no state data
	 * - should store the auxilliary data in the buffer
	 * - should return the amount of amount stored
	 * - not permitted to return an error
	 * - the netfs data from the cookie being used as the source is
	 *   presented
	 */
	uint16_t (*get_aux)(const void *cookie_netfs_data,
			    void *buffer,
			    uint16_t bufmax);

	/* consult the netfs about the state of an object
	 * - this function can be absent if the index carries no state data
	 * - the netfs data from the cookie being used as the target is
	 *   presented, as is the auxilliary data
	 */
	enum fscache_checkaux (*check_aux)(void *cookie_netfs_data,
					   const void *data,
					   uint16_t datalen);

	/* get an extra reference on a read context
	 * - this function can be absent if the completion function doesn't
	 *   require a context
	 */
	void (*get_context)(void *cookie_netfs_data, void *context);

	/* release an extra reference on a read context
	 * - this function can be absent if the completion function doesn't
	 *   require a context
	 */
	void (*put_context)(void *cookie_netfs_data, void *context);

	/* indicate pages that now have cache metadata retained
	 * - this function should mark the specified pages as now being cached
	 * - the pages will have been marked with PG_fscache before this is
	 *   called, so this is optional
	 */
	void (*mark_pages_cached)(void *cookie_netfs_data,
				  struct address_space *mapping,
				  struct pagevec *cached_pvec);

	/* indicate the cookie is no longer cached
	 * - this function is called when the backing store currently caching
	 *   a cookie is removed
	 * - the netfs should use this to clean up any markers indicating
	 *   cached pages
	 * - this is mandatory for any object that may have data
	 */
	void (*now_uncached)(void *cookie_netfs_data);
};

struct fscache_netfs {
	uint32_t			version;	/* indexing version */
	const char			*name;		/* filesystem name */
	struct fscache_cookie		*primary_index;
	struct list_head		link;		/* internal link */
};

extern int __fscache_register_netfs(struct fscache_netfs *);
extern void __fscache_unregister_netfs(struct fscache_netfs *);
extern struct fscache_cache_tag *__fscache_lookup_cache_tag(const char *);
extern void __fscache_release_cache_tag(struct fscache_cache_tag *);

extern struct fscache_cookie *__fscache_acquire_cookie(
	struct fscache_cookie *,
	const struct fscache_cookie_def *,
	void *);
extern void __fscache_relinquish_cookie(struct fscache_cookie *, int);
extern void __fscache_update_cookie(struct fscache_cookie *);
extern int __fscache_attr_changed(struct fscache_cookie *);
extern int __fscache_read_or_alloc_page(struct fscache_cookie *,
					struct page *,
					fscache_rw_complete_t,
					void *,
					gfp_t);
extern int __fscache_read_or_alloc_pages(struct fscache_cookie *,
					 struct address_space *,
					 struct list_head *,
					 unsigned *,
					 fscache_rw_complete_t,
					 void *,
					 gfp_t);
extern int __fscache_alloc_page(struct fscache_cookie *, struct page *, gfp_t);
extern int __fscache_write_page(struct fscache_cookie *, struct page *, gfp_t);
extern void __fscache_uncache_page(struct fscache_cookie *, struct page *);
extern bool __fscache_check_page_write(struct fscache_cookie *, struct page *);
extern void __fscache_wait_on_page_write(struct fscache_cookie *, struct page *);
extern bool __fscache_maybe_release_page(struct fscache_cookie *, struct page *,
					 gfp_t);

static inline
int fscache_register_netfs(struct fscache_netfs *netfs)
{
	if (fscache_available())
		return __fscache_register_netfs(netfs);
	else
		return 0;
}

static inline
void fscache_unregister_netfs(struct fscache_netfs *netfs)
{
	if (fscache_available())
		__fscache_unregister_netfs(netfs);
}

static inline
struct fscache_cache_tag *fscache_lookup_cache_tag(const char *name)
{
	if (fscache_available())
		return __fscache_lookup_cache_tag(name);
	else
		return NULL;
}

static inline
void fscache_release_cache_tag(struct fscache_cache_tag *tag)
{
	if (fscache_available())
		__fscache_release_cache_tag(tag);
}

static inline
struct fscache_cookie *fscache_acquire_cookie(
	struct fscache_cookie *parent,
	const struct fscache_cookie_def *def,
	void *netfs_data)
{
	if (fscache_cookie_valid(parent))
		return __fscache_acquire_cookie(parent, def, netfs_data);
	else
		return NULL;
}

static inline
void fscache_relinquish_cookie(struct fscache_cookie *cookie, int retire)
{
	if (fscache_cookie_valid(cookie))
		__fscache_relinquish_cookie(cookie, retire);
}

static inline
void fscache_update_cookie(struct fscache_cookie *cookie)
{
	if (fscache_cookie_valid(cookie))
		__fscache_update_cookie(cookie);
}

static inline
int fscache_pin_cookie(struct fscache_cookie *cookie)
{
	return -ENOBUFS;
}

static inline
void fscache_unpin_cookie(struct fscache_cookie *cookie)
{
}

static inline
int fscache_attr_changed(struct fscache_cookie *cookie)
{
	if (fscache_cookie_valid(cookie))
		return __fscache_attr_changed(cookie);
	else
		return -ENOBUFS;
}

static inline
int fscache_reserve_space(struct fscache_cookie *cookie, loff_t size)
{
	return -ENOBUFS;
}

static inline
int fscache_read_or_alloc_page(struct fscache_cookie *cookie,
			       struct page *page,
			       fscache_rw_complete_t end_io_func,
			       void *context,
			       gfp_t gfp)
{
	if (fscache_cookie_valid(cookie))
		return __fscache_read_or_alloc_page(cookie, page, end_io_func,
						    context, gfp);
	else
		return -ENOBUFS;
}

static inline
int fscache_read_or_alloc_pages(struct fscache_cookie *cookie,
				struct address_space *mapping,
				struct list_head *pages,
				unsigned *nr_pages,
				fscache_rw_complete_t end_io_func,
				void *context,
				gfp_t gfp)
{
	if (fscache_cookie_valid(cookie))
		return __fscache_read_or_alloc_pages(cookie, mapping, pages,
						     nr_pages, end_io_func,
						     context, gfp);
	else
		return -ENOBUFS;
}

static inline
int fscache_alloc_page(struct fscache_cookie *cookie,
		       struct page *page,
		       gfp_t gfp)
{
	if (fscache_cookie_valid(cookie))
		return __fscache_alloc_page(cookie, page, gfp);
	else
		return -ENOBUFS;
}

static inline
int fscache_write_page(struct fscache_cookie *cookie,
		       struct page *page,
		       gfp_t gfp)
{
	if (fscache_cookie_valid(cookie))
		return __fscache_write_page(cookie, page, gfp);
	else
		return -ENOBUFS;
}

static inline
void fscache_uncache_page(struct fscache_cookie *cookie,
			  struct page *page)
{
	if (fscache_cookie_valid(cookie))
		__fscache_uncache_page(cookie, page);
}

static inline
bool fscache_check_page_write(struct fscache_cookie *cookie,
			      struct page *page)
{
	if (fscache_cookie_valid(cookie))
		return __fscache_check_page_write(cookie, page);
	return false;
}

static inline
void fscache_wait_on_page_write(struct fscache_cookie *cookie,
				struct page *page)
{
	if (fscache_cookie_valid(cookie))
		__fscache_wait_on_page_write(cookie, page);
}

static inline
bool fscache_maybe_release_page(struct fscache_cookie *cookie,
				struct page *page,
				gfp_t gfp)
{
	if (fscache_cookie_valid(cookie) && PageFsCache(page))
		return __fscache_maybe_release_page(cookie, page, gfp);
	return false;
}

#endif /* _LINUX_FSCACHE_H */
