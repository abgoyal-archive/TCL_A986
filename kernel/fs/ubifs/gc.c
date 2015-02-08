


#include <linux/slab.h>
#include <linux/pagemap.h>
#include <linux/list_sort.h>
#include "ubifs.h"

#define SOFT_LEBS_LIMIT 4
#define HARD_LEBS_LIMIT 32

static int switch_gc_head(struct ubifs_info *c)
{
	int err, gc_lnum = c->gc_lnum;
	struct ubifs_wbuf *wbuf = &c->jheads[GCHD].wbuf;

	ubifs_assert(gc_lnum != -1);
	dbg_gc("switch GC head from LEB %d:%d to LEB %d (waste %d bytes)",
	       wbuf->lnum, wbuf->offs + wbuf->used, gc_lnum,
	       c->leb_size - wbuf->offs - wbuf->used);

	err = ubifs_wbuf_sync_nolock(wbuf);
	if (err)
		return err;

	/*
	 * The GC write-buffer was synchronized, we may safely unmap
	 * 'c->gc_lnum'.
	 */
	err = ubifs_leb_unmap(c, gc_lnum);
	if (err)
		return err;

	err = ubifs_add_bud_to_log(c, GCHD, gc_lnum, 0);
	if (err)
		return err;

	c->gc_lnum = -1;
	err = ubifs_wbuf_seek_nolock(wbuf, gc_lnum, 0, UBI_LONGTERM);
	return err;
}

int data_nodes_cmp(void *priv, struct list_head *a, struct list_head *b)
{
	ino_t inuma, inumb;
	struct ubifs_info *c = priv;
	struct ubifs_scan_node *sa, *sb;

	cond_resched();
	sa = list_entry(a, struct ubifs_scan_node, list);
	sb = list_entry(b, struct ubifs_scan_node, list);
	ubifs_assert(key_type(c, &sa->key) == UBIFS_DATA_KEY);
	ubifs_assert(key_type(c, &sb->key) == UBIFS_DATA_KEY);

	inuma = key_inum(c, &sa->key);
	inumb = key_inum(c, &sb->key);

	if (inuma == inumb) {
		unsigned int blka = key_block(c, &sa->key);
		unsigned int blkb = key_block(c, &sb->key);

		if (blka <= blkb)
			return -1;
	} else if (inuma <= inumb)
		return -1;

	return 1;
}

int nondata_nodes_cmp(void *priv, struct list_head *a, struct list_head *b)
{
	int typea, typeb;
	ino_t inuma, inumb;
	struct ubifs_info *c = priv;
	struct ubifs_scan_node *sa, *sb;

	cond_resched();
	sa = list_entry(a, struct ubifs_scan_node, list);
	sb = list_entry(b, struct ubifs_scan_node, list);
	typea = key_type(c, &sa->key);
	typeb = key_type(c, &sb->key);
	ubifs_assert(typea != UBIFS_DATA_KEY && typeb != UBIFS_DATA_KEY);

	/* Inodes go before directory entries */
	if (typea == UBIFS_INO_KEY) {
		if (typeb == UBIFS_INO_KEY)
			return sb->len - sa->len;
		return -1;
	}
	if (typeb == UBIFS_INO_KEY)
		return 1;

	ubifs_assert(typea == UBIFS_DENT_KEY && typeb == UBIFS_DENT_KEY);
	inuma = key_inum(c, &sa->key);
	inumb = key_inum(c, &sb->key);

	if (inuma == inumb) {
		uint32_t hasha = key_hash(c, &sa->key);
		uint32_t hashb = key_hash(c, &sb->key);

		if (hasha <= hashb)
			return -1;
	} else if (inuma <= inumb)
		return -1;

	return 1;
}

static int sort_nodes(struct ubifs_info *c, struct ubifs_scan_leb *sleb,
		      struct list_head *nondata, int *min)
{
	struct ubifs_scan_node *snod, *tmp;

	*min = INT_MAX;

	/* Separate data nodes and non-data nodes */
	list_for_each_entry_safe(snod, tmp, &sleb->nodes, list) {
		int err;

		ubifs_assert(snod->type != UBIFS_IDX_NODE);
		ubifs_assert(snod->type != UBIFS_REF_NODE);
		ubifs_assert(snod->type != UBIFS_CS_NODE);

		err = ubifs_tnc_has_node(c, &snod->key, 0, sleb->lnum,
					 snod->offs, 0);
		if (err < 0)
			return err;

		if (!err) {
			/* The node is obsolete, remove it from the list */
			list_del(&snod->list);
			kfree(snod);
			continue;
		}

		if (snod->len < *min)
			*min = snod->len;

		if (key_type(c, &snod->key) != UBIFS_DATA_KEY)
			list_move_tail(&snod->list, nondata);
	}

	/* Sort data and non-data nodes */
	list_sort(c, &sleb->nodes, &data_nodes_cmp);
	list_sort(c, nondata, &nondata_nodes_cmp);
	return 0;
}

static int move_node(struct ubifs_info *c, struct ubifs_scan_leb *sleb,
		     struct ubifs_scan_node *snod, struct ubifs_wbuf *wbuf)
{
	int err, new_lnum = wbuf->lnum, new_offs = wbuf->offs + wbuf->used;

	cond_resched();
	err = ubifs_wbuf_write_nolock(wbuf, snod->node, snod->len);
	if (err)
		return err;

	err = ubifs_tnc_replace(c, &snod->key, sleb->lnum,
				snod->offs, new_lnum, new_offs,
				snod->len);
	list_del(&snod->list);
	kfree(snod);
	return err;
}

static int move_nodes(struct ubifs_info *c, struct ubifs_scan_leb *sleb)
{
	int err, min;
	LIST_HEAD(nondata);
	struct ubifs_wbuf *wbuf = &c->jheads[GCHD].wbuf;

	if (wbuf->lnum == -1) {
		/*
		 * The GC journal head is not set, because it is the first GC
		 * invocation since mount.
		 */
		err = switch_gc_head(c);
		if (err)
			return err;
	}

	err = sort_nodes(c, sleb, &nondata, &min);
	if (err)
		goto out;

	/* Write nodes to their new location. Use the first-fit strategy */
	while (1) {
		int avail;
		struct ubifs_scan_node *snod, *tmp;

		/* Move data nodes */
		list_for_each_entry_safe(snod, tmp, &sleb->nodes, list) {
			avail = c->leb_size - wbuf->offs - wbuf->used;
			if  (snod->len > avail)
				/*
				 * Do not skip data nodes in order to optimize
				 * bulk-read.
				 */
				break;

			err = move_node(c, sleb, snod, wbuf);
			if (err)
				goto out;
		}

		/* Move non-data nodes */
		list_for_each_entry_safe(snod, tmp, &nondata, list) {
			avail = c->leb_size - wbuf->offs - wbuf->used;
			if (avail < min)
				break;

			if  (snod->len > avail) {
				/*
				 * Keep going only if this is an inode with
				 * some data. Otherwise stop and switch the GC
				 * head. IOW, we assume that data-less inode
				 * nodes and direntry nodes are roughly of the
				 * same size.
				 */
				if (key_type(c, &snod->key) == UBIFS_DENT_KEY ||
				    snod->len == UBIFS_INO_NODE_SZ)
					break;
				continue;
			}

			err = move_node(c, sleb, snod, wbuf);
			if (err)
				goto out;
		}

		if (list_empty(&sleb->nodes) && list_empty(&nondata))
			break;

		/*
		 * Waste the rest of the space in the LEB and switch to the
		 * next LEB.
		 */
		err = switch_gc_head(c);
		if (err)
			goto out;
	}

	return 0;

out:
	list_splice_tail(&nondata, &sleb->nodes);
	return err;
}

static int gc_sync_wbufs(struct ubifs_info *c)
{
	int err, i;

	for (i = 0; i < c->jhead_cnt; i++) {
		if (i == GCHD)
			continue;
		err = ubifs_wbuf_sync(&c->jheads[i].wbuf);
		if (err)
			return err;
	}
	return 0;
}

int ubifs_garbage_collect_leb(struct ubifs_info *c, struct ubifs_lprops *lp)
{
	struct ubifs_scan_leb *sleb;
	struct ubifs_scan_node *snod;
	struct ubifs_wbuf *wbuf = &c->jheads[GCHD].wbuf;
	int err = 0, lnum = lp->lnum;

	ubifs_assert(c->gc_lnum != -1 || wbuf->offs + wbuf->used == 0 ||
		     c->need_recovery);
	ubifs_assert(c->gc_lnum != lnum);
	ubifs_assert(wbuf->lnum != lnum);

	/*
	 * We scan the entire LEB even though we only really need to scan up to
	 * (c->leb_size - lp->free).
	 */
	sleb = ubifs_scan(c, lnum, 0, c->sbuf, 0);
	if (IS_ERR(sleb))
		return PTR_ERR(sleb);

	ubifs_assert(!list_empty(&sleb->nodes));
	snod = list_entry(sleb->nodes.next, struct ubifs_scan_node, list);

	if (snod->type == UBIFS_IDX_NODE) {
		struct ubifs_gced_idx_leb *idx_gc;

		dbg_gc("indexing LEB %d (free %d, dirty %d)",
		       lnum, lp->free, lp->dirty);
		list_for_each_entry(snod, &sleb->nodes, list) {
			struct ubifs_idx_node *idx = snod->node;
			int level = le16_to_cpu(idx->level);

			ubifs_assert(snod->type == UBIFS_IDX_NODE);
			key_read(c, ubifs_idx_key(c, idx), &snod->key);
			err = ubifs_dirty_idx_node(c, &snod->key, level, lnum,
						   snod->offs);
			if (err)
				goto out;
		}

		idx_gc = kmalloc(sizeof(struct ubifs_gced_idx_leb), GFP_NOFS);
		if (!idx_gc) {
			err = -ENOMEM;
			goto out;
		}

		idx_gc->lnum = lnum;
		idx_gc->unmap = 0;
		list_add(&idx_gc->list, &c->idx_gc);

		/*
		 * Don't release the LEB until after the next commit, because
		 * it may contain data which is needed for recovery. So
		 * although we freed this LEB, it will become usable only after
		 * the commit.
		 */
		err = ubifs_change_one_lp(c, lnum, c->leb_size, 0, 0,
					  LPROPS_INDEX, 1);
		if (err)
			goto out;
		err = LEB_FREED_IDX;
	} else {
		dbg_gc("data LEB %d (free %d, dirty %d)",
		       lnum, lp->free, lp->dirty);

		err = move_nodes(c, sleb);
		if (err)
			goto out_inc_seq;

		err = gc_sync_wbufs(c);
		if (err)
			goto out_inc_seq;

		err = ubifs_change_one_lp(c, lnum, c->leb_size, 0, 0, 0, 0);
		if (err)
			goto out_inc_seq;

		/* Allow for races with TNC */
		c->gced_lnum = lnum;
		smp_wmb();
		c->gc_seq += 1;
		smp_wmb();

		if (c->gc_lnum == -1) {
			c->gc_lnum = lnum;
			err = LEB_RETAINED;
		} else {
			err = ubifs_wbuf_sync_nolock(wbuf);
			if (err)
				goto out;

			err = ubifs_leb_unmap(c, lnum);
			if (err)
				goto out;

			err = LEB_FREED;
		}
	}

out:
	ubifs_scan_destroy(sleb);
	return err;

out_inc_seq:
	/* We may have moved at least some nodes so allow for races with TNC */
	c->gced_lnum = lnum;
	smp_wmb();
	c->gc_seq += 1;
	smp_wmb();
	goto out;
}

int ubifs_garbage_collect(struct ubifs_info *c, int anyway)
{
	int i, err, ret, min_space = c->dead_wm;
	struct ubifs_lprops lp;
	struct ubifs_wbuf *wbuf = &c->jheads[GCHD].wbuf;

	ubifs_assert_cmt_locked(c);

	if (ubifs_gc_should_commit(c))
		return -EAGAIN;

	mutex_lock_nested(&wbuf->io_mutex, wbuf->jhead);

	if (c->ro_media) {
		ret = -EROFS;
		goto out_unlock;
	}

	/* We expect the write-buffer to be empty on entry */
	ubifs_assert(!wbuf->used);

	for (i = 0; ; i++) {
		int space_before = c->leb_size - wbuf->offs - wbuf->used;
		int space_after;

		cond_resched();

		/* Give the commit an opportunity to run */
		if (ubifs_gc_should_commit(c)) {
			ret = -EAGAIN;
			break;
		}

		if (i > SOFT_LEBS_LIMIT && !list_empty(&c->idx_gc)) {
			/*
			 * We've done enough iterations. Indexing LEBs were
			 * moved and will be available after the commit.
			 */
			dbg_gc("soft limit, some index LEBs GC'ed, -EAGAIN");
			ubifs_commit_required(c);
			ret = -EAGAIN;
			break;
		}

		if (i > HARD_LEBS_LIMIT) {
			/*
			 * We've moved too many LEBs and have not made
			 * progress, give up.
			 */
			dbg_gc("hard limit, -ENOSPC");
			ret = -ENOSPC;
			break;
		}

		/*
		 * Empty and freeable LEBs can turn up while we waited for
		 * the wbuf lock, or while we have been running GC. In that
		 * case, we should just return one of those instead of
		 * continuing to GC dirty LEBs. Hence we request
		 * 'ubifs_find_dirty_leb()' to return an empty LEB if it can.
		 */
		ret = ubifs_find_dirty_leb(c, &lp, min_space, anyway ? 0 : 1);
		if (ret) {
			if (ret == -ENOSPC)
				dbg_gc("no more dirty LEBs");
			break;
		}

		dbg_gc("found LEB %d: free %d, dirty %d, sum %d "
		       "(min. space %d)", lp.lnum, lp.free, lp.dirty,
		       lp.free + lp.dirty, min_space);

		if (lp.free + lp.dirty == c->leb_size) {
			/* An empty LEB was returned */
			dbg_gc("LEB %d is free, return it", lp.lnum);
			/*
			 * ubifs_find_dirty_leb() doesn't return freeable index
			 * LEBs.
			 */
			ubifs_assert(!(lp.flags & LPROPS_INDEX));
			if (lp.free != c->leb_size) {
				/*
				 * Write buffers must be sync'd before
				 * unmapping freeable LEBs, because one of them
				 * may contain data which obsoletes something
				 * in 'lp.pnum'.
				 */
				ret = gc_sync_wbufs(c);
				if (ret)
					goto out;
				ret = ubifs_change_one_lp(c, lp.lnum,
							  c->leb_size, 0, 0, 0,
							  0);
				if (ret)
					goto out;
			}
			ret = ubifs_leb_unmap(c, lp.lnum);
			if (ret)
				goto out;
			ret = lp.lnum;
			break;
		}

		space_before = c->leb_size - wbuf->offs - wbuf->used;
		if (wbuf->lnum == -1)
			space_before = 0;

		ret = ubifs_garbage_collect_leb(c, &lp);
		if (ret < 0) {
			if (ret == -EAGAIN || ret == -ENOSPC) {
				/*
				 * These codes are not errors, so we have to
				 * return the LEB to lprops. But if the
				 * 'ubifs_return_leb()' function fails, its
				 * failure code is propagated to the caller
				 * instead of the original '-EAGAIN' or
				 * '-ENOSPC'.
				 */
				err = ubifs_return_leb(c, lp.lnum);
				if (err)
					ret = err;
				break;
			}
			goto out;
		}

		if (ret == LEB_FREED) {
			/* An LEB has been freed and is ready for use */
			dbg_gc("LEB %d freed, return", lp.lnum);
			ret = lp.lnum;
			break;
		}

		if (ret == LEB_FREED_IDX) {
			/*
			 * This was an indexing LEB and it cannot be
			 * immediately used. And instead of requesting the
			 * commit straight away, we try to garbage collect some
			 * more.
			 */
			dbg_gc("indexing LEB %d freed, continue", lp.lnum);
			continue;
		}

		ubifs_assert(ret == LEB_RETAINED);
		space_after = c->leb_size - wbuf->offs - wbuf->used;
		dbg_gc("LEB %d retained, freed %d bytes", lp.lnum,
		       space_after - space_before);

		if (space_after > space_before) {
			/* GC makes progress, keep working */
			min_space >>= 1;
			if (min_space < c->dead_wm)
				min_space = c->dead_wm;
			continue;
		}

		dbg_gc("did not make progress");

		/*
		 * GC moved an LEB bud have not done any progress. This means
		 * that the previous GC head LEB contained too few free space
		 * and the LEB which was GC'ed contained only large nodes which
		 * did not fit that space.
		 *
		 * We can do 2 things:
		 * 1. pick another LEB in a hope it'll contain a small node
		 *    which will fit the space we have at the end of current GC
		 *    head LEB, but there is no guarantee, so we try this out
		 *    unless we have already been working for too long;
		 * 2. request an LEB with more dirty space, which will force
		 *    'ubifs_find_dirty_leb()' to start scanning the lprops
		 *    table, instead of just picking one from the heap
		 *    (previously it already picked the dirtiest LEB).
		 */
		if (i < SOFT_LEBS_LIMIT) {
			dbg_gc("try again");
			continue;
		}

		min_space <<= 1;
		if (min_space > c->dark_wm)
			min_space = c->dark_wm;
		dbg_gc("set min. space to %d", min_space);
	}

	if (ret == -ENOSPC && !list_empty(&c->idx_gc)) {
		dbg_gc("no space, some index LEBs GC'ed, -EAGAIN");
		ubifs_commit_required(c);
		ret = -EAGAIN;
	}

	err = ubifs_wbuf_sync_nolock(wbuf);
	if (!err)
		err = ubifs_leb_unmap(c, c->gc_lnum);
	if (err) {
		ret = err;
		goto out;
	}
out_unlock:
	mutex_unlock(&wbuf->io_mutex);
	return ret;

out:
	ubifs_assert(ret < 0);
	ubifs_assert(ret != -ENOSPC && ret != -EAGAIN);
	ubifs_ro_mode(c, ret);
	ubifs_wbuf_sync_nolock(wbuf);
	mutex_unlock(&wbuf->io_mutex);
	ubifs_return_leb(c, lp.lnum);
	return ret;
}

int ubifs_gc_start_commit(struct ubifs_info *c)
{
	struct ubifs_gced_idx_leb *idx_gc;
	const struct ubifs_lprops *lp;
	int err = 0, flags;

	ubifs_get_lprops(c);

	/*
	 * Unmap (non-index) freeable LEBs. Note that recovery requires that all
	 * wbufs are sync'd before this, which is done in 'do_commit()'.
	 */
	while (1) {
		lp = ubifs_fast_find_freeable(c);
		if (IS_ERR(lp)) {
			err = PTR_ERR(lp);
			goto out;
		}
		if (!lp)
			break;
		ubifs_assert(!(lp->flags & LPROPS_TAKEN));
		ubifs_assert(!(lp->flags & LPROPS_INDEX));
		err = ubifs_leb_unmap(c, lp->lnum);
		if (err)
			goto out;
		lp = ubifs_change_lp(c, lp, c->leb_size, 0, lp->flags, 0);
		if (IS_ERR(lp)) {
			err = PTR_ERR(lp);
			goto out;
		}
		ubifs_assert(!(lp->flags & LPROPS_TAKEN));
		ubifs_assert(!(lp->flags & LPROPS_INDEX));
	}

	/* Mark GC'd index LEBs OK to unmap after this commit finishes */
	list_for_each_entry(idx_gc, &c->idx_gc, list)
		idx_gc->unmap = 1;

	/* Record index freeable LEBs for unmapping after commit */
	while (1) {
		lp = ubifs_fast_find_frdi_idx(c);
		if (IS_ERR(lp)) {
			err = PTR_ERR(lp);
			goto out;
		}
		if (!lp)
			break;
		idx_gc = kmalloc(sizeof(struct ubifs_gced_idx_leb), GFP_NOFS);
		if (!idx_gc) {
			err = -ENOMEM;
			goto out;
		}
		ubifs_assert(!(lp->flags & LPROPS_TAKEN));
		ubifs_assert(lp->flags & LPROPS_INDEX);
		/* Don't release the LEB until after the next commit */
		flags = (lp->flags | LPROPS_TAKEN) ^ LPROPS_INDEX;
		lp = ubifs_change_lp(c, lp, c->leb_size, 0, flags, 1);
		if (IS_ERR(lp)) {
			err = PTR_ERR(lp);
			kfree(idx_gc);
			goto out;
		}
		ubifs_assert(lp->flags & LPROPS_TAKEN);
		ubifs_assert(!(lp->flags & LPROPS_INDEX));
		idx_gc->lnum = lp->lnum;
		idx_gc->unmap = 1;
		list_add(&idx_gc->list, &c->idx_gc);
	}
out:
	ubifs_release_lprops(c);
	return err;
}

int ubifs_gc_end_commit(struct ubifs_info *c)
{
	struct ubifs_gced_idx_leb *idx_gc, *tmp;
	struct ubifs_wbuf *wbuf;
	int err = 0;

	wbuf = &c->jheads[GCHD].wbuf;
	mutex_lock_nested(&wbuf->io_mutex, wbuf->jhead);
	list_for_each_entry_safe(idx_gc, tmp, &c->idx_gc, list)
		if (idx_gc->unmap) {
			dbg_gc("LEB %d", idx_gc->lnum);
			err = ubifs_leb_unmap(c, idx_gc->lnum);
			if (err)
				goto out;
			err = ubifs_change_one_lp(c, idx_gc->lnum, LPROPS_NC,
					  LPROPS_NC, 0, LPROPS_TAKEN, -1);
			if (err)
				goto out;
			list_del(&idx_gc->list);
			kfree(idx_gc);
		}
out:
	mutex_unlock(&wbuf->io_mutex);
	return err;
}

void ubifs_destroy_idx_gc(struct ubifs_info *c)
{
	while (!list_empty(&c->idx_gc)) {
		struct ubifs_gced_idx_leb *idx_gc;

		idx_gc = list_entry(c->idx_gc.next, struct ubifs_gced_idx_leb,
				    list);
		c->idx_gc_cnt -= 1;
		list_del(&idx_gc->list);
		kfree(idx_gc);
	}
}

int ubifs_get_idx_gc_leb(struct ubifs_info *c)
{
	struct ubifs_gced_idx_leb *idx_gc;
	int lnum;

	if (list_empty(&c->idx_gc))
		return -ENOSPC;
	idx_gc = list_entry(c->idx_gc.next, struct ubifs_gced_idx_leb, list);
	lnum = idx_gc->lnum;
	/* c->idx_gc_cnt is updated by the caller when lprops are updated */
	list_del(&idx_gc->list);
	kfree(idx_gc);
	return lnum;
}
