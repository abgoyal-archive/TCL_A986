

#ifndef _NILFS_SUFILE_H
#define _NILFS_SUFILE_H

#include <linux/fs.h>
#include <linux/buffer_head.h>
#include <linux/nilfs2_fs.h>
#include "mdt.h"


static inline unsigned long nilfs_sufile_get_nsegments(struct inode *sufile)
{
	return NILFS_MDT(sufile)->mi_nilfs->ns_nsegments;
}

unsigned long nilfs_sufile_get_ncleansegs(struct inode *sufile);

int nilfs_sufile_alloc(struct inode *, __u64 *);
int nilfs_sufile_mark_dirty(struct inode *sufile, __u64 segnum);
int nilfs_sufile_set_segment_usage(struct inode *sufile, __u64 segnum,
				   unsigned long nblocks, time_t modtime);
int nilfs_sufile_get_stat(struct inode *, struct nilfs_sustat *);
ssize_t nilfs_sufile_get_suinfo(struct inode *, __u64, void *, unsigned,
				size_t);

int nilfs_sufile_updatev(struct inode *, __u64 *, size_t, int, size_t *,
			 void (*dofunc)(struct inode *, __u64,
					struct buffer_head *,
					struct buffer_head *));
int nilfs_sufile_update(struct inode *, __u64, int,
			void (*dofunc)(struct inode *, __u64,
				       struct buffer_head *,
				       struct buffer_head *));
void nilfs_sufile_do_scrap(struct inode *, __u64, struct buffer_head *,
			   struct buffer_head *);
void nilfs_sufile_do_free(struct inode *, __u64, struct buffer_head *,
			  struct buffer_head *);
void nilfs_sufile_do_cancel_free(struct inode *, __u64, struct buffer_head *,
				 struct buffer_head *);
void nilfs_sufile_do_set_error(struct inode *, __u64, struct buffer_head *,
			       struct buffer_head *);

int nilfs_sufile_read(struct inode *sufile, struct nilfs_inode *raw_inode);
struct inode *nilfs_sufile_new(struct the_nilfs *nilfs, size_t susize);

static inline int nilfs_sufile_scrap(struct inode *sufile, __u64 segnum)
{
	return nilfs_sufile_update(sufile, segnum, 1, nilfs_sufile_do_scrap);
}

static inline int nilfs_sufile_free(struct inode *sufile, __u64 segnum)
{
	return nilfs_sufile_update(sufile, segnum, 0, nilfs_sufile_do_free);
}

static inline int nilfs_sufile_freev(struct inode *sufile, __u64 *segnumv,
				     size_t nsegs, size_t *ndone)
{
	return nilfs_sufile_updatev(sufile, segnumv, nsegs, 0, ndone,
				    nilfs_sufile_do_free);
}

static inline int nilfs_sufile_cancel_freev(struct inode *sufile,
					    __u64 *segnumv, size_t nsegs,
					    size_t *ndone)
{
	return nilfs_sufile_updatev(sufile, segnumv, nsegs, 0, ndone,
				    nilfs_sufile_do_cancel_free);
}

static inline int nilfs_sufile_set_error(struct inode *sufile, __u64 segnum)
{
	return nilfs_sufile_update(sufile, segnum, 0,
				   nilfs_sufile_do_set_error);
}

#endif	/* _NILFS_SUFILE_H */
