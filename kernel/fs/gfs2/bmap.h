

#ifndef __BMAP_DOT_H__
#define __BMAP_DOT_H__

#include "inode.h"

struct inode;
struct gfs2_inode;
struct page;



static inline void gfs2_write_calc_reserv(const struct gfs2_inode *ip,
					  unsigned int len,
					  unsigned int *data_blocks,
					  unsigned int *ind_blocks)
{
	const struct gfs2_sbd *sdp = GFS2_SB(&ip->i_inode);
	unsigned int tmp;

	BUG_ON(gfs2_is_dir(ip));
	*data_blocks = (len >> sdp->sd_sb.sb_bsize_shift) + 3;
	*ind_blocks = 3 * (sdp->sd_max_height - 1);

	for (tmp = *data_blocks; tmp > sdp->sd_diptrs;) {
		tmp = DIV_ROUND_UP(tmp, sdp->sd_inptrs);
		*ind_blocks += tmp;
	}
}

int gfs2_unstuff_dinode(struct gfs2_inode *ip, struct page *page);
int gfs2_block_map(struct inode *inode, sector_t lblock, struct buffer_head *bh, int create);
int gfs2_extent_map(struct inode *inode, u64 lblock, int *new, u64 *dblock, unsigned *extlen);

int gfs2_truncatei(struct gfs2_inode *ip, u64 size);
int gfs2_truncatei_resume(struct gfs2_inode *ip);
int gfs2_file_dealloc(struct gfs2_inode *ip);
int gfs2_write_alloc_required(struct gfs2_inode *ip, u64 offset,
			      unsigned int len, int *alloc_required);

#endif /* __BMAP_DOT_H__ */
