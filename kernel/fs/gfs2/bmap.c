

#include <linux/spinlock.h>
#include <linux/completion.h>
#include <linux/buffer_head.h>
#include <linux/gfs2_ondisk.h>
#include <linux/crc32.h>

#include "gfs2.h"
#include "incore.h"
#include "bmap.h"
#include "glock.h"
#include "inode.h"
#include "meta_io.h"
#include "quota.h"
#include "rgrp.h"
#include "trans.h"
#include "dir.h"
#include "util.h"
#include "trace_gfs2.h"

struct metapath {
	struct buffer_head *mp_bh[GFS2_MAX_META_HEIGHT];
	__u16 mp_list[GFS2_MAX_META_HEIGHT];
};

typedef int (*block_call_t) (struct gfs2_inode *ip, struct buffer_head *dibh,
			     struct buffer_head *bh, __be64 *top,
			     __be64 *bottom, unsigned int height,
			     void *data);

struct strip_mine {
	int sm_first;
	unsigned int sm_height;
};


static int gfs2_unstuffer_page(struct gfs2_inode *ip, struct buffer_head *dibh,
			       u64 block, struct page *page)
{
	struct inode *inode = &ip->i_inode;
	struct buffer_head *bh;
	int release = 0;

	if (!page || page->index) {
		page = grab_cache_page(inode->i_mapping, 0);
		if (!page)
			return -ENOMEM;
		release = 1;
	}

	if (!PageUptodate(page)) {
		void *kaddr = kmap(page);
		u64 dsize = i_size_read(inode);
 
		if (dsize > (dibh->b_size - sizeof(struct gfs2_dinode)))
			dsize = dibh->b_size - sizeof(struct gfs2_dinode);

		memcpy(kaddr, dibh->b_data + sizeof(struct gfs2_dinode), dsize);
		memset(kaddr + dsize, 0, PAGE_CACHE_SIZE - dsize);
		kunmap(page);

		SetPageUptodate(page);
	}

	if (!page_has_buffers(page))
		create_empty_buffers(page, 1 << inode->i_blkbits,
				     (1 << BH_Uptodate));

	bh = page_buffers(page);

	if (!buffer_mapped(bh))
		map_bh(bh, inode->i_sb, block);

	set_buffer_uptodate(bh);
	if (!gfs2_is_jdata(ip))
		mark_buffer_dirty(bh);
	if (!gfs2_is_writeback(ip))
		gfs2_trans_add_bh(ip->i_gl, bh, 0);

	if (release) {
		unlock_page(page);
		page_cache_release(page);
	}

	return 0;
}


int gfs2_unstuff_dinode(struct gfs2_inode *ip, struct page *page)
{
	struct buffer_head *bh, *dibh;
	struct gfs2_dinode *di;
	u64 block = 0;
	int isdir = gfs2_is_dir(ip);
	int error;

	down_write(&ip->i_rw_mutex);

	error = gfs2_meta_inode_buffer(ip, &dibh);
	if (error)
		goto out;

	if (ip->i_disksize) {
		/* Get a free block, fill it with the stuffed data,
		   and write it out to disk */

		unsigned int n = 1;
		error = gfs2_alloc_block(ip, &block, &n);
		if (error)
			goto out_brelse;
		if (isdir) {
			gfs2_trans_add_unrevoke(GFS2_SB(&ip->i_inode), block, 1);
			error = gfs2_dir_get_new_buffer(ip, block, &bh);
			if (error)
				goto out_brelse;
			gfs2_buffer_copy_tail(bh, sizeof(struct gfs2_meta_header),
					      dibh, sizeof(struct gfs2_dinode));
			brelse(bh);
		} else {
			error = gfs2_unstuffer_page(ip, dibh, block, page);
			if (error)
				goto out_brelse;
		}
	}

	/*  Set up the pointer to the new block  */

	gfs2_trans_add_bh(ip->i_gl, dibh, 1);
	di = (struct gfs2_dinode *)dibh->b_data;
	gfs2_buffer_clear_tail(dibh, sizeof(struct gfs2_dinode));

	if (ip->i_disksize) {
		*(__be64 *)(di + 1) = cpu_to_be64(block);
		gfs2_add_inode_blocks(&ip->i_inode, 1);
		di->di_blocks = cpu_to_be64(gfs2_get_inode_blocks(&ip->i_inode));
	}

	ip->i_height = 1;
	di->di_height = cpu_to_be16(1);

out_brelse:
	brelse(dibh);
out:
	up_write(&ip->i_rw_mutex);
	return error;
}



static void find_metapath(const struct gfs2_sbd *sdp, u64 block,
			  struct metapath *mp, unsigned int height)
{
	unsigned int i;

	for (i = height; i--;)
		mp->mp_list[i] = do_div(block, sdp->sd_inptrs);

}

static inline unsigned int metapath_branch_start(const struct metapath *mp)
{
	if (mp->mp_list[0] == 0)
		return 2;
	return 1;
}


static inline __be64 *metapointer(unsigned int height, const struct metapath *mp)
{
	struct buffer_head *bh = mp->mp_bh[height];
	unsigned int head_size = (height > 0) ?
		sizeof(struct gfs2_meta_header) : sizeof(struct gfs2_dinode);
	return ((__be64 *)(bh->b_data + head_size)) + mp->mp_list[height];
}


static int lookup_metapath(struct gfs2_inode *ip, struct metapath *mp)
{
	unsigned int end_of_metadata = ip->i_height - 1;
	unsigned int x;
	__be64 *ptr;
	u64 dblock;
	int ret;

	for (x = 0; x < end_of_metadata; x++) {
		ptr = metapointer(x, mp);
		dblock = be64_to_cpu(*ptr);
		if (!dblock)
			return x + 1;

		ret = gfs2_meta_indirect_buffer(ip, x+1, dblock, 0, &mp->mp_bh[x+1]);
		if (ret)
			return ret;
	}

	return ip->i_height;
}

static inline void release_metapath(struct metapath *mp)
{
	int i;

	for (i = 0; i < GFS2_MAX_META_HEIGHT; i++) {
		if (mp->mp_bh[i] == NULL)
			break;
		brelse(mp->mp_bh[i]);
	}
}


static inline unsigned int gfs2_extent_length(void *start, unsigned int len, __be64 *ptr, unsigned limit, int *eob)
{
	const __be64 *end = (start + len);
	const __be64 *first = ptr;
	u64 d = be64_to_cpu(*ptr);

	*eob = 0;
	do {
		ptr++;
		if (ptr >= end)
			break;
		if (limit && --limit == 0)
			break;
		if (d)
			d++;
	} while(be64_to_cpu(*ptr) == d);
	if (ptr >= end)
		*eob = 1;
	return (ptr - first);
}

static inline void bmap_lock(struct gfs2_inode *ip, int create)
{
	if (create)
		down_write(&ip->i_rw_mutex);
	else
		down_read(&ip->i_rw_mutex);
}

static inline void bmap_unlock(struct gfs2_inode *ip, int create)
{
	if (create)
		up_write(&ip->i_rw_mutex);
	else
		up_read(&ip->i_rw_mutex);
}

static inline __be64 *gfs2_indirect_init(struct metapath *mp,
					 struct gfs2_glock *gl, unsigned int i,
					 unsigned offset, u64 bn)
{
	__be64 *ptr = (__be64 *)(mp->mp_bh[i - 1]->b_data +
		       ((i > 1) ? sizeof(struct gfs2_meta_header) :
				 sizeof(struct gfs2_dinode)));
	BUG_ON(i < 1);
	BUG_ON(mp->mp_bh[i] != NULL);
	mp->mp_bh[i] = gfs2_meta_new(gl, bn);
	gfs2_trans_add_bh(gl, mp->mp_bh[i], 1);
	gfs2_metatype_set(mp->mp_bh[i], GFS2_METATYPE_IN, GFS2_FORMAT_IN);
	gfs2_buffer_clear_tail(mp->mp_bh[i], sizeof(struct gfs2_meta_header));
	ptr += offset;
	*ptr = cpu_to_be64(bn);
	return ptr;
}

enum alloc_state {
	ALLOC_DATA = 0,
	ALLOC_GROW_DEPTH = 1,
	ALLOC_GROW_HEIGHT = 2,
	/* ALLOC_UNSTUFF = 3,   TBD and rather complicated */
};


static int gfs2_bmap_alloc(struct inode *inode, const sector_t lblock,
			   struct buffer_head *bh_map, struct metapath *mp,
			   const unsigned int sheight,
			   const unsigned int height,
			   const unsigned int maxlen)
{
	struct gfs2_inode *ip = GFS2_I(inode);
	struct gfs2_sbd *sdp = GFS2_SB(inode);
	struct buffer_head *dibh = mp->mp_bh[0];
	u64 bn, dblock = 0;
	unsigned n, i, blks, alloced = 0, iblks = 0, branch_start = 0;
	unsigned dblks = 0;
	unsigned ptrs_per_blk;
	const unsigned end_of_metadata = height - 1;
	int eob = 0;
	enum alloc_state state;
	__be64 *ptr;
	__be64 zero_bn = 0;

	BUG_ON(sheight < 1);
	BUG_ON(dibh == NULL);

	gfs2_trans_add_bh(ip->i_gl, dibh, 1);

	if (height == sheight) {
		struct buffer_head *bh;
		/* Bottom indirect block exists, find unalloced extent size */
		ptr = metapointer(end_of_metadata, mp);
		bh = mp->mp_bh[end_of_metadata];
		dblks = gfs2_extent_length(bh->b_data, bh->b_size, ptr, maxlen,
					   &eob);
		BUG_ON(dblks < 1);
		state = ALLOC_DATA;
	} else {
		/* Need to allocate indirect blocks */
		ptrs_per_blk = height > 1 ? sdp->sd_inptrs : sdp->sd_diptrs;
		dblks = min(maxlen, ptrs_per_blk - mp->mp_list[end_of_metadata]);
		if (height == ip->i_height) {
			/* Writing into existing tree, extend tree down */
			iblks = height - sheight;
			state = ALLOC_GROW_DEPTH;
		} else {
			/* Building up tree height */
			state = ALLOC_GROW_HEIGHT;
			iblks = height - ip->i_height;
			branch_start = metapath_branch_start(mp);
			iblks += (height - branch_start);
		}
	}

	/* start of the second part of the function (state machine) */

	blks = dblks + iblks;
	i = sheight;
	do {
		int error;
		n = blks - alloced;
		error = gfs2_alloc_block(ip, &bn, &n);
		if (error)
			return error;
		alloced += n;
		if (state != ALLOC_DATA || gfs2_is_jdata(ip))
			gfs2_trans_add_unrevoke(sdp, bn, n);
		switch (state) {
		/* Growing height of tree */
		case ALLOC_GROW_HEIGHT:
			if (i == 1) {
				ptr = (__be64 *)(dibh->b_data +
						 sizeof(struct gfs2_dinode));
				zero_bn = *ptr;
			}
			for (; i - 1 < height - ip->i_height && n > 0; i++, n--)
				gfs2_indirect_init(mp, ip->i_gl, i, 0, bn++);
			if (i - 1 == height - ip->i_height) {
				i--;
				gfs2_buffer_copy_tail(mp->mp_bh[i],
						sizeof(struct gfs2_meta_header),
						dibh, sizeof(struct gfs2_dinode));
				gfs2_buffer_clear_tail(dibh,
						sizeof(struct gfs2_dinode) +
						sizeof(__be64));
				ptr = (__be64 *)(mp->mp_bh[i]->b_data +
					sizeof(struct gfs2_meta_header));
				*ptr = zero_bn;
				state = ALLOC_GROW_DEPTH;
				for(i = branch_start; i < height; i++) {
					if (mp->mp_bh[i] == NULL)
						break;
					brelse(mp->mp_bh[i]);
					mp->mp_bh[i] = NULL;
				}
				i = branch_start;
			}
			if (n == 0)
				break;
		/* Branching from existing tree */
		case ALLOC_GROW_DEPTH:
			if (i > 1 && i < height)
				gfs2_trans_add_bh(ip->i_gl, mp->mp_bh[i-1], 1);
			for (; i < height && n > 0; i++, n--)
				gfs2_indirect_init(mp, ip->i_gl, i,
						   mp->mp_list[i-1], bn++);
			if (i == height)
				state = ALLOC_DATA;
			if (n == 0)
				break;
		/* Tree complete, adding data blocks */
		case ALLOC_DATA:
			BUG_ON(n > dblks);
			BUG_ON(mp->mp_bh[end_of_metadata] == NULL);
			gfs2_trans_add_bh(ip->i_gl, mp->mp_bh[end_of_metadata], 1);
			dblks = n;
			ptr = metapointer(end_of_metadata, mp);
			dblock = bn;
			while (n-- > 0)
				*ptr++ = cpu_to_be64(bn++);
			break;
		}
	} while ((state != ALLOC_DATA) || !dblock);

	ip->i_height = height;
	gfs2_add_inode_blocks(&ip->i_inode, alloced);
	gfs2_dinode_out(ip, mp->mp_bh[0]->b_data);
	map_bh(bh_map, inode->i_sb, dblock);
	bh_map->b_size = dblks << inode->i_blkbits;
	set_buffer_new(bh_map);
	return 0;
}


int gfs2_block_map(struct inode *inode, sector_t lblock,
		   struct buffer_head *bh_map, int create)
{
	struct gfs2_inode *ip = GFS2_I(inode);
	struct gfs2_sbd *sdp = GFS2_SB(inode);
	unsigned int bsize = sdp->sd_sb.sb_bsize;
	const unsigned int maxlen = bh_map->b_size >> inode->i_blkbits;
	const u64 *arr = sdp->sd_heightsize;
	__be64 *ptr;
	u64 size;
	struct metapath mp;
	int ret;
	int eob;
	unsigned int len;
	struct buffer_head *bh;
	u8 height;

	BUG_ON(maxlen == 0);

	memset(mp.mp_bh, 0, sizeof(mp.mp_bh));
	bmap_lock(ip, create);
	clear_buffer_mapped(bh_map);
	clear_buffer_new(bh_map);
	clear_buffer_boundary(bh_map);
	trace_gfs2_bmap(ip, bh_map, lblock, create, 1);
	if (gfs2_is_dir(ip)) {
		bsize = sdp->sd_jbsize;
		arr = sdp->sd_jheightsize;
	}

	ret = gfs2_meta_inode_buffer(ip, &mp.mp_bh[0]);
	if (ret)
		goto out;

	height = ip->i_height;
	size = (lblock + 1) * bsize;
	while (size > arr[height])
		height++;
	find_metapath(sdp, lblock, &mp, height);
	ret = 1;
	if (height > ip->i_height || gfs2_is_stuffed(ip))
		goto do_alloc;
	ret = lookup_metapath(ip, &mp);
	if (ret < 0)
		goto out;
	if (ret != ip->i_height)
		goto do_alloc;
	ptr = metapointer(ip->i_height - 1, &mp);
	if (*ptr == 0)
		goto do_alloc;
	map_bh(bh_map, inode->i_sb, be64_to_cpu(*ptr));
	bh = mp.mp_bh[ip->i_height - 1];
	len = gfs2_extent_length(bh->b_data, bh->b_size, ptr, maxlen, &eob);
	bh_map->b_size = (len << inode->i_blkbits);
	if (eob)
		set_buffer_boundary(bh_map);
	ret = 0;
out:
	release_metapath(&mp);
	trace_gfs2_bmap(ip, bh_map, lblock, create, ret);
	bmap_unlock(ip, create);
	return ret;

do_alloc:
	/* All allocations are done here, firstly check create flag */
	if (!create) {
		BUG_ON(gfs2_is_stuffed(ip));
		ret = 0;
		goto out;
	}

	/* At this point ret is the tree depth of already allocated blocks */
	ret = gfs2_bmap_alloc(inode, lblock, bh_map, &mp, ret, height, maxlen);
	goto out;
}

int gfs2_extent_map(struct inode *inode, u64 lblock, int *new, u64 *dblock, unsigned *extlen)
{
	struct buffer_head bh = { .b_state = 0, .b_blocknr = 0 };
	int ret;
	int create = *new;

	BUG_ON(!extlen);
	BUG_ON(!dblock);
	BUG_ON(!new);

	bh.b_size = 1 << (inode->i_blkbits + (create ? 0 : 5));
	ret = gfs2_block_map(inode, lblock, &bh, create);
	*extlen = bh.b_size >> inode->i_blkbits;
	*dblock = bh.b_blocknr;
	if (buffer_new(&bh))
		*new = 1;
	else
		*new = 0;
	return ret;
}


static int recursive_scan(struct gfs2_inode *ip, struct buffer_head *dibh,
			  struct metapath *mp, unsigned int height,
			  u64 block, int first, block_call_t bc,
			  void *data)
{
	struct gfs2_sbd *sdp = GFS2_SB(&ip->i_inode);
	struct buffer_head *bh = NULL;
	__be64 *top, *bottom;
	u64 bn;
	int error;
	int mh_size = sizeof(struct gfs2_meta_header);

	if (!height) {
		error = gfs2_meta_inode_buffer(ip, &bh);
		if (error)
			return error;
		dibh = bh;

		top = (__be64 *)(bh->b_data + sizeof(struct gfs2_dinode)) + mp->mp_list[0];
		bottom = (__be64 *)(bh->b_data + sizeof(struct gfs2_dinode)) + sdp->sd_diptrs;
	} else {
		error = gfs2_meta_indirect_buffer(ip, height, block, 0, &bh);
		if (error)
			return error;

		top = (__be64 *)(bh->b_data + mh_size) +
				  (first ? mp->mp_list[height] : 0);

		bottom = (__be64 *)(bh->b_data + mh_size) + sdp->sd_inptrs;
	}

	error = bc(ip, dibh, bh, top, bottom, height, data);
	if (error)
		goto out;

	if (height < ip->i_height - 1)
		for (; top < bottom; top++, first = 0) {
			if (!*top)
				continue;

			bn = be64_to_cpu(*top);

			error = recursive_scan(ip, dibh, mp, height + 1, bn,
					       first, bc, data);
			if (error)
				break;
		}

out:
	brelse(bh);
	return error;
}


static int do_strip(struct gfs2_inode *ip, struct buffer_head *dibh,
		    struct buffer_head *bh, __be64 *top, __be64 *bottom,
		    unsigned int height, void *data)
{
	struct strip_mine *sm = data;
	struct gfs2_sbd *sdp = GFS2_SB(&ip->i_inode);
	struct gfs2_rgrp_list rlist;
	u64 bn, bstart;
	u32 blen;
	__be64 *p;
	unsigned int rg_blocks = 0;
	int metadata;
	unsigned int revokes = 0;
	int x;
	int error;

	if (!*top)
		sm->sm_first = 0;

	if (height != sm->sm_height)
		return 0;

	if (sm->sm_first) {
		top++;
		sm->sm_first = 0;
	}

	metadata = (height != ip->i_height - 1);
	if (metadata)
		revokes = (height) ? sdp->sd_inptrs : sdp->sd_diptrs;

	error = gfs2_rindex_hold(sdp, &ip->i_alloc->al_ri_gh);
	if (error)
		return error;

	memset(&rlist, 0, sizeof(struct gfs2_rgrp_list));
	bstart = 0;
	blen = 0;

	for (p = top; p < bottom; p++) {
		if (!*p)
			continue;

		bn = be64_to_cpu(*p);

		if (bstart + blen == bn)
			blen++;
		else {
			if (bstart)
				gfs2_rlist_add(sdp, &rlist, bstart);

			bstart = bn;
			blen = 1;
		}
	}

	if (bstart)
		gfs2_rlist_add(sdp, &rlist, bstart);
	else
		goto out; /* Nothing to do */

	gfs2_rlist_alloc(&rlist, LM_ST_EXCLUSIVE);

	for (x = 0; x < rlist.rl_rgrps; x++) {
		struct gfs2_rgrpd *rgd;
		rgd = rlist.rl_ghs[x].gh_gl->gl_object;
		rg_blocks += rgd->rd_length;
	}

	error = gfs2_glock_nq_m(rlist.rl_rgrps, rlist.rl_ghs);
	if (error)
		goto out_rlist;

	error = gfs2_trans_begin(sdp, rg_blocks + RES_DINODE +
				 RES_INDIRECT + RES_STATFS + RES_QUOTA,
				 revokes);
	if (error)
		goto out_rg_gunlock;

	down_write(&ip->i_rw_mutex);

	gfs2_trans_add_bh(ip->i_gl, dibh, 1);
	gfs2_trans_add_bh(ip->i_gl, bh, 1);

	bstart = 0;
	blen = 0;

	for (p = top; p < bottom; p++) {
		if (!*p)
			continue;

		bn = be64_to_cpu(*p);

		if (bstart + blen == bn)
			blen++;
		else {
			if (bstart) {
				if (metadata)
					gfs2_free_meta(ip, bstart, blen);
				else
					gfs2_free_data(ip, bstart, blen);
			}

			bstart = bn;
			blen = 1;
		}

		*p = 0;
		gfs2_add_inode_blocks(&ip->i_inode, -1);
	}
	if (bstart) {
		if (metadata)
			gfs2_free_meta(ip, bstart, blen);
		else
			gfs2_free_data(ip, bstart, blen);
	}

	ip->i_inode.i_mtime = ip->i_inode.i_ctime = CURRENT_TIME;

	gfs2_dinode_out(ip, dibh->b_data);

	up_write(&ip->i_rw_mutex);

	gfs2_trans_end(sdp);

out_rg_gunlock:
	gfs2_glock_dq_m(rlist.rl_rgrps, rlist.rl_ghs);
out_rlist:
	gfs2_rlist_free(&rlist);
out:
	gfs2_glock_dq_uninit(&ip->i_alloc->al_ri_gh);
	return error;
}


static int do_grow(struct gfs2_inode *ip, u64 size)
{
	struct gfs2_sbd *sdp = GFS2_SB(&ip->i_inode);
	struct gfs2_alloc *al;
	struct buffer_head *dibh;
	int error;

	al = gfs2_alloc_get(ip);
	if (!al)
		return -ENOMEM;

	error = gfs2_quota_lock_check(ip);
	if (error)
		goto out;

	al->al_requested = sdp->sd_max_height + RES_DATA;

	error = gfs2_inplace_reserve(ip);
	if (error)
		goto out_gunlock_q;

	error = gfs2_trans_begin(sdp,
			sdp->sd_max_height + al->al_rgd->rd_length +
			RES_JDATA + RES_DINODE + RES_STATFS + RES_QUOTA, 0);
	if (error)
		goto out_ipres;

	error = gfs2_meta_inode_buffer(ip, &dibh);
	if (error)
		goto out_end_trans;

	if (size > sdp->sd_sb.sb_bsize - sizeof(struct gfs2_dinode)) {
		if (gfs2_is_stuffed(ip)) {
			error = gfs2_unstuff_dinode(ip, NULL);
			if (error)
				goto out_brelse;
		}
	}

	ip->i_disksize = size;
	ip->i_inode.i_mtime = ip->i_inode.i_ctime = CURRENT_TIME;
	gfs2_trans_add_bh(ip->i_gl, dibh, 1);
	gfs2_dinode_out(ip, dibh->b_data);

out_brelse:
	brelse(dibh);
out_end_trans:
	gfs2_trans_end(sdp);
out_ipres:
	gfs2_inplace_release(ip);
out_gunlock_q:
	gfs2_quota_unlock(ip);
out:
	gfs2_alloc_put(ip);
	return error;
}


static int gfs2_block_truncate_page(struct address_space *mapping)
{
	struct inode *inode = mapping->host;
	struct gfs2_inode *ip = GFS2_I(inode);
	loff_t from = inode->i_size;
	unsigned long index = from >> PAGE_CACHE_SHIFT;
	unsigned offset = from & (PAGE_CACHE_SIZE-1);
	unsigned blocksize, iblock, length, pos;
	struct buffer_head *bh;
	struct page *page;
	int err;

	page = grab_cache_page(mapping, index);
	if (!page)
		return 0;

	blocksize = inode->i_sb->s_blocksize;
	length = blocksize - (offset & (blocksize - 1));
	iblock = index << (PAGE_CACHE_SHIFT - inode->i_sb->s_blocksize_bits);

	if (!page_has_buffers(page))
		create_empty_buffers(page, blocksize, 0);

	/* Find the buffer that contains "offset" */
	bh = page_buffers(page);
	pos = blocksize;
	while (offset >= pos) {
		bh = bh->b_this_page;
		iblock++;
		pos += blocksize;
	}

	err = 0;

	if (!buffer_mapped(bh)) {
		gfs2_block_map(inode, iblock, bh, 0);
		/* unmapped? It's a hole - nothing to do */
		if (!buffer_mapped(bh))
			goto unlock;
	}

	/* Ok, it's mapped. Make sure it's up-to-date */
	if (PageUptodate(page))
		set_buffer_uptodate(bh);

	if (!buffer_uptodate(bh)) {
		err = -EIO;
		ll_rw_block(READ, 1, &bh);
		wait_on_buffer(bh);
		/* Uhhuh. Read error. Complain and punt. */
		if (!buffer_uptodate(bh))
			goto unlock;
		err = 0;
	}

	if (!gfs2_is_writeback(ip))
		gfs2_trans_add_bh(ip->i_gl, bh, 0);

	zero_user(page, offset, length);
	mark_buffer_dirty(bh);
unlock:
	unlock_page(page);
	page_cache_release(page);
	return err;
}

static int trunc_start(struct gfs2_inode *ip, u64 size)
{
	struct gfs2_sbd *sdp = GFS2_SB(&ip->i_inode);
	struct buffer_head *dibh;
	int journaled = gfs2_is_jdata(ip);
	int error;

	error = gfs2_trans_begin(sdp,
				 RES_DINODE + (journaled ? RES_JDATA : 0), 0);
	if (error)
		return error;

	error = gfs2_meta_inode_buffer(ip, &dibh);
	if (error)
		goto out;

	if (gfs2_is_stuffed(ip)) {
		u64 dsize = size + sizeof(struct gfs2_inode);
		ip->i_disksize = size;
		ip->i_inode.i_mtime = ip->i_inode.i_ctime = CURRENT_TIME;
		gfs2_trans_add_bh(ip->i_gl, dibh, 1);
		gfs2_dinode_out(ip, dibh->b_data);
		if (dsize > dibh->b_size)
			dsize = dibh->b_size;
		gfs2_buffer_clear_tail(dibh, dsize);
		error = 1;
	} else {
		if (size & (u64)(sdp->sd_sb.sb_bsize - 1))
			error = gfs2_block_truncate_page(ip->i_inode.i_mapping);

		if (!error) {
			ip->i_disksize = size;
			ip->i_inode.i_mtime = ip->i_inode.i_ctime = CURRENT_TIME;
			ip->i_diskflags |= GFS2_DIF_TRUNC_IN_PROG;
			gfs2_trans_add_bh(ip->i_gl, dibh, 1);
			gfs2_dinode_out(ip, dibh->b_data);
		}
	}

	brelse(dibh);

out:
	gfs2_trans_end(sdp);
	return error;
}

static int trunc_dealloc(struct gfs2_inode *ip, u64 size)
{
	struct gfs2_sbd *sdp = GFS2_SB(&ip->i_inode);
	unsigned int height = ip->i_height;
	u64 lblock;
	struct metapath mp;
	int error;

	if (!size)
		lblock = 0;
	else
		lblock = (size - 1) >> sdp->sd_sb.sb_bsize_shift;

	find_metapath(sdp, lblock, &mp, ip->i_height);
	if (!gfs2_alloc_get(ip))
		return -ENOMEM;

	error = gfs2_quota_hold(ip, NO_QUOTA_CHANGE, NO_QUOTA_CHANGE);
	if (error)
		goto out;

	while (height--) {
		struct strip_mine sm;
		sm.sm_first = !!size;
		sm.sm_height = height;

		error = recursive_scan(ip, NULL, &mp, 0, 0, 1, do_strip, &sm);
		if (error)
			break;
	}

	gfs2_quota_unhold(ip);

out:
	gfs2_alloc_put(ip);
	return error;
}

static int trunc_end(struct gfs2_inode *ip)
{
	struct gfs2_sbd *sdp = GFS2_SB(&ip->i_inode);
	struct buffer_head *dibh;
	int error;

	error = gfs2_trans_begin(sdp, RES_DINODE, 0);
	if (error)
		return error;

	down_write(&ip->i_rw_mutex);

	error = gfs2_meta_inode_buffer(ip, &dibh);
	if (error)
		goto out;

	if (!ip->i_disksize) {
		ip->i_height = 0;
		ip->i_goal = ip->i_no_addr;
		gfs2_buffer_clear_tail(dibh, sizeof(struct gfs2_dinode));
	}
	ip->i_inode.i_mtime = ip->i_inode.i_ctime = CURRENT_TIME;
	ip->i_diskflags &= ~GFS2_DIF_TRUNC_IN_PROG;

	gfs2_trans_add_bh(ip->i_gl, dibh, 1);
	gfs2_dinode_out(ip, dibh->b_data);
	brelse(dibh);

out:
	up_write(&ip->i_rw_mutex);
	gfs2_trans_end(sdp);
	return error;
}


static int do_shrink(struct gfs2_inode *ip, u64 size)
{
	int error;

	error = trunc_start(ip, size);
	if (error < 0)
		return error;
	if (error > 0)
		return 0;

	error = trunc_dealloc(ip, size);
	if (!error)
		error = trunc_end(ip);

	return error;
}

static int do_touch(struct gfs2_inode *ip, u64 size)
{
	struct gfs2_sbd *sdp = GFS2_SB(&ip->i_inode);
	struct buffer_head *dibh;
	int error;

	error = gfs2_trans_begin(sdp, RES_DINODE, 0);
	if (error)
		return error;

	down_write(&ip->i_rw_mutex);

	error = gfs2_meta_inode_buffer(ip, &dibh);
	if (error)
		goto do_touch_out;

	ip->i_inode.i_mtime = ip->i_inode.i_ctime = CURRENT_TIME;
	gfs2_trans_add_bh(ip->i_gl, dibh, 1);
	gfs2_dinode_out(ip, dibh->b_data);
	brelse(dibh);

do_touch_out:
	up_write(&ip->i_rw_mutex);
	gfs2_trans_end(sdp);
	return error;
}


int gfs2_truncatei(struct gfs2_inode *ip, u64 size)
{
	int error;

	if (gfs2_assert_warn(GFS2_SB(&ip->i_inode), S_ISREG(ip->i_inode.i_mode)))
		return -EINVAL;

	if (size > ip->i_disksize)
		error = do_grow(ip, size);
	else if (size < ip->i_disksize)
		error = do_shrink(ip, size);
	else
		/* update time stamps */
		error = do_touch(ip, size);

	return error;
}

int gfs2_truncatei_resume(struct gfs2_inode *ip)
{
	int error;
	error = trunc_dealloc(ip, ip->i_disksize);
	if (!error)
		error = trunc_end(ip);
	return error;
}

int gfs2_file_dealloc(struct gfs2_inode *ip)
{
	return trunc_dealloc(ip, 0);
}


int gfs2_write_alloc_required(struct gfs2_inode *ip, u64 offset,
			      unsigned int len, int *alloc_required)
{
	struct gfs2_sbd *sdp = GFS2_SB(&ip->i_inode);
	struct buffer_head bh;
	unsigned int shift;
	u64 lblock, lblock_stop, size;
	u64 end_of_file;

	*alloc_required = 0;

	if (!len)
		return 0;

	if (gfs2_is_stuffed(ip)) {
		if (offset + len >
		    sdp->sd_sb.sb_bsize - sizeof(struct gfs2_dinode))
			*alloc_required = 1;
		return 0;
	}

	*alloc_required = 1;
	shift = sdp->sd_sb.sb_bsize_shift;
	BUG_ON(gfs2_is_dir(ip));
	end_of_file = (ip->i_disksize + sdp->sd_sb.sb_bsize - 1) >> shift;
	lblock = offset >> shift;
	lblock_stop = (offset + len + sdp->sd_sb.sb_bsize - 1) >> shift;
	if (lblock_stop > end_of_file)
		return 0;

	size = (lblock_stop - lblock) << shift;
	do {
		bh.b_state = 0;
		bh.b_size = size;
		gfs2_block_map(&ip->i_inode, lblock, &bh, 0);
		if (!buffer_mapped(&bh))
			return 0;
		size -= bh.b_size;
		lblock += (bh.b_size >> ip->i_inode.i_blkbits);
	} while(size > 0);

	*alloc_required = 0;
	return 0;
}

