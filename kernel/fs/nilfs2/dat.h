

#ifndef _NILFS_DAT_H
#define _NILFS_DAT_H

#include <linux/types.h>
#include <linux/buffer_head.h>
#include <linux/fs.h>


struct nilfs_palloc_req;

int nilfs_dat_translate(struct inode *, __u64, sector_t *);

int nilfs_dat_prepare_alloc(struct inode *, struct nilfs_palloc_req *);
void nilfs_dat_commit_alloc(struct inode *, struct nilfs_palloc_req *);
void nilfs_dat_abort_alloc(struct inode *, struct nilfs_palloc_req *);
int nilfs_dat_prepare_start(struct inode *, struct nilfs_palloc_req *);
void nilfs_dat_commit_start(struct inode *, struct nilfs_palloc_req *,
			    sector_t);
int nilfs_dat_prepare_end(struct inode *, struct nilfs_palloc_req *);
void nilfs_dat_commit_end(struct inode *, struct nilfs_palloc_req *, int);
void nilfs_dat_abort_end(struct inode *, struct nilfs_palloc_req *);
int nilfs_dat_prepare_update(struct inode *, struct nilfs_palloc_req *,
			     struct nilfs_palloc_req *);
void nilfs_dat_commit_update(struct inode *, struct nilfs_palloc_req *,
			     struct nilfs_palloc_req *, int);
void nilfs_dat_abort_update(struct inode *, struct nilfs_palloc_req *,
			    struct nilfs_palloc_req *);

int nilfs_dat_mark_dirty(struct inode *, __u64);
int nilfs_dat_freev(struct inode *, __u64 *, size_t);
int nilfs_dat_move(struct inode *, __u64, sector_t);
ssize_t nilfs_dat_get_vinfo(struct inode *, void *, unsigned, size_t);

int nilfs_dat_read(struct inode *dat, struct nilfs_inode *raw_inode);
struct inode *nilfs_dat_new(struct the_nilfs *nilfs, size_t entry_size);

#endif	/* _NILFS_DAT_H */
