

#ifndef _CRYPTO_SCATTERWALK_H
#define _CRYPTO_SCATTERWALK_H
#include <linux/mm.h>
#include <asm/scatterlist.h>

struct scatter_walk {
	struct scatterlist	*sg;
	struct page		*page;
	void			*data;
	unsigned int		len_this_page;
	unsigned int		len_this_segment;
	unsigned int		offset;
};

static inline struct scatterlist *sg_next(struct scatterlist *sg)
{
	return sg + 1;
}

static inline int scatterwalk_samebuf(struct scatter_walk *walk_in,
				      struct scatter_walk *walk_out,
				      void *src_p, void *dst_p)
{
	return walk_in->page == walk_out->page &&
	       walk_in->offset == walk_out->offset &&
	       walk_in->data == src_p && walk_out->data == dst_p;
}

void *scatterwalk_whichbuf(struct scatter_walk *walk, unsigned int nbytes, void *scratch);
void scatterwalk_start(struct scatter_walk *walk, struct scatterlist *sg);
int scatterwalk_copychunks(void *buf, struct scatter_walk *walk, size_t nbytes, int out);
void scatterwalk_map(struct scatter_walk *walk, int out);
void scatterwalk_done(struct scatter_walk *walk, int out, int more);

#endif  /* _CRYPTO_SCATTERWALK_H */
