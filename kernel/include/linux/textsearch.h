
#ifndef __LINUX_TEXTSEARCH_H
#define __LINUX_TEXTSEARCH_H

#include <linux/types.h>
#include <linux/list.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/slab.h>

struct ts_config;

#define TS_AUTOLOAD	1 /* Automatically load textsearch modules when needed */
#define TS_IGNORECASE	2 /* Searches string case insensitively */

struct ts_state
{
	unsigned int		offset;
	char			cb[40];
};

struct ts_ops
{
	const char		*name;
	struct ts_config *	(*init)(const void *, unsigned int, gfp_t, int);
	unsigned int		(*find)(struct ts_config *,
					struct ts_state *);
	void			(*destroy)(struct ts_config *);
	void *			(*get_pattern)(struct ts_config *);
	unsigned int		(*get_pattern_len)(struct ts_config *);
	struct module		*owner;
	struct list_head	list;
};

struct ts_config
{
	struct ts_ops		*ops;
	int 			flags;

	/**
	 * get_next_block - fetch next block of data
	 * @consumed: number of bytes consumed by the caller
	 * @dst: destination buffer
	 * @conf: search configuration
	 * @state: search state
	 *
	 * Called repeatedly until 0 is returned. Must assign the
	 * head of the next block of data to &*dst and return the length
	 * of the block or 0 if at the end. consumed == 0 indicates
	 * a new search. May store/read persistent values in state->cb.
	 */
	unsigned int		(*get_next_block)(unsigned int consumed,
						  const u8 **dst,
						  struct ts_config *conf,
						  struct ts_state *state);

	/**
	 * finish - finalize/clean a series of get_next_block() calls
	 * @conf: search configuration
	 * @state: search state
	 *
	 * Called after the last use of get_next_block(), may be used
	 * to cleanup any leftovers.
	 */
	void			(*finish)(struct ts_config *conf,
					  struct ts_state *state);
};

static inline unsigned int textsearch_next(struct ts_config *conf,
					   struct ts_state *state)
{
	unsigned int ret = conf->ops->find(conf, state);

	if (conf->finish)
		conf->finish(conf, state);

	return ret;
}

static inline unsigned int textsearch_find(struct ts_config *conf,
					   struct ts_state *state)
{
	state->offset = 0;
	return textsearch_next(conf, state);
}

static inline void *textsearch_get_pattern(struct ts_config *conf)
{
	return conf->ops->get_pattern(conf);
}

static inline unsigned int textsearch_get_pattern_len(struct ts_config *conf)
{
	return conf->ops->get_pattern_len(conf);
}

extern int textsearch_register(struct ts_ops *);
extern int textsearch_unregister(struct ts_ops *);
extern struct ts_config *textsearch_prepare(const char *, const void *,
					    unsigned int, gfp_t, int);
extern void textsearch_destroy(struct ts_config *conf);
extern unsigned int textsearch_find_continuous(struct ts_config *,
					       struct ts_state *,
					       const void *, unsigned int);


#define TS_PRIV_ALIGNTO	8
#define TS_PRIV_ALIGN(len) (((len) + TS_PRIV_ALIGNTO-1) & ~(TS_PRIV_ALIGNTO-1))

static inline struct ts_config *alloc_ts_config(size_t payload,
						gfp_t gfp_mask)
{
	struct ts_config *conf;

	conf = kzalloc(TS_PRIV_ALIGN(sizeof(*conf)) + payload, gfp_mask);
	if (conf == NULL)
		return ERR_PTR(-ENOMEM);

	return conf;
}

static inline void *ts_config_priv(struct ts_config *conf)
{
	return ((u8 *) conf + TS_PRIV_ALIGN(sizeof(struct ts_config)));
}

#endif
