


#include "ubifs.h"
#include <linux/writeback.h>
#include <linux/math64.h>

#define MAX_MKSPC_RETRIES 3

#define NR_TO_WRITE 16

static void shrink_liability(struct ubifs_info *c, int nr_to_write)
{
	down_read(&c->vfs_sb->s_umount);
	writeback_inodes_sb(c->vfs_sb);
	up_read(&c->vfs_sb->s_umount);
}

static int run_gc(struct ubifs_info *c)
{
	int err, lnum;

	/* Make some free space by garbage-collecting dirty space */
	down_read(&c->commit_sem);
	lnum = ubifs_garbage_collect(c, 1);
	up_read(&c->commit_sem);
	if (lnum < 0)
		return lnum;

	/* GC freed one LEB, return it to lprops */
	dbg_budg("GC freed LEB %d", lnum);
	err = ubifs_return_leb(c, lnum);
	if (err)
		return err;
	return 0;
}

static long long get_liability(struct ubifs_info *c)
{
	long long liab;

	spin_lock(&c->space_lock);
	liab = c->budg_idx_growth + c->budg_data_growth + c->budg_dd_growth;
	spin_unlock(&c->space_lock);
	return liab;
}

static int make_free_space(struct ubifs_info *c)
{
	int err, retries = 0;
	long long liab1, liab2;

	do {
		liab1 = get_liability(c);
		/*
		 * We probably have some dirty pages or inodes (liability), try
		 * to write them back.
		 */
		dbg_budg("liability %lld, run write-back", liab1);
		shrink_liability(c, NR_TO_WRITE);

		liab2 = get_liability(c);
		if (liab2 < liab1)
			return -EAGAIN;

		dbg_budg("new liability %lld (not shrinked)", liab2);

		/* Liability did not shrink again, try GC */
		dbg_budg("Run GC");
		err = run_gc(c);
		if (!err)
			return -EAGAIN;

		if (err != -EAGAIN && err != -ENOSPC)
			/* Some real error happened */
			return err;

		dbg_budg("Run commit (retries %d)", retries);
		err = ubifs_run_commit(c);
		if (err)
			return err;
	} while (retries++ < MAX_MKSPC_RETRIES);

	return -ENOSPC;
}

int ubifs_calc_min_idx_lebs(struct ubifs_info *c)
{
	int idx_lebs;
	long long idx_size;

	idx_size = c->old_idx_sz + c->budg_idx_growth + c->budg_uncommitted_idx;
	/* And make sure we have thrice the index size of space reserved */
	idx_size += idx_size << 1;
	/*
	 * We do not maintain 'old_idx_size' as 'old_idx_lebs'/'old_idx_bytes'
	 * pair, nor similarly the two variables for the new index size, so we
	 * have to do this costly 64-bit division on fast-path.
	 */
	idx_lebs = div_u64(idx_size + c->idx_leb_size - 1, c->idx_leb_size);
	/*
	 * The index head is not available for the in-the-gaps method, so add an
	 * extra LEB to compensate.
	 */
	idx_lebs += 1;
	if (idx_lebs < MIN_INDEX_LEBS)
		idx_lebs = MIN_INDEX_LEBS;
	return idx_lebs;
}

long long ubifs_calc_available(const struct ubifs_info *c, int min_idx_lebs)
{
	int subtract_lebs;
	long long available;

	available = c->main_bytes - c->lst.total_used;

	/*
	 * Now 'available' contains theoretically available flash space
	 * assuming there is no index, so we have to subtract the space which
	 * is reserved for the index.
	 */
	subtract_lebs = min_idx_lebs;

	/* Take into account that GC reserves one LEB for its own needs */
	subtract_lebs += 1;

	/*
	 * The GC journal head LEB is not really accessible. And since
	 * different write types go to different heads, we may count only on
	 * one head's space.
	 */
	subtract_lebs += c->jhead_cnt - 1;

	/* We also reserve one LEB for deletions, which bypass budgeting */
	subtract_lebs += 1;

	available -= (long long)subtract_lebs * c->leb_size;

	/* Subtract the dead space which is not available for use */
	available -= c->lst.total_dead;

	/*
	 * Subtract dark space, which might or might not be usable - it depends
	 * on the data which we have on the media and which will be written. If
	 * this is a lot of uncompressed or not-compressible data, the dark
	 * space cannot be used.
	 */
	available -= c->lst.total_dark;

	/*
	 * However, there is more dark space. The index may be bigger than
	 * @min_idx_lebs. Those extra LEBs are assumed to be available, but
	 * their dark space is not included in total_dark, so it is subtracted
	 * here.
	 */
	if (c->lst.idx_lebs > min_idx_lebs) {
		subtract_lebs = c->lst.idx_lebs - min_idx_lebs;
		available -= subtract_lebs * c->dark_wm;
	}

	/* The calculations are rough and may end up with a negative number */
	return available > 0 ? available : 0;
}

static int can_use_rp(struct ubifs_info *c)
{
	if (current_fsuid() == c->rp_uid || capable(CAP_SYS_RESOURCE) ||
	    (c->rp_gid != 0 && in_group_p(c->rp_gid)))
		return 1;
	return 0;
}

static int do_budget_space(struct ubifs_info *c)
{
	long long outstanding, available;
	int lebs, rsvd_idx_lebs, min_idx_lebs;

	/* First budget index space */
	min_idx_lebs = ubifs_calc_min_idx_lebs(c);

	/* Now 'min_idx_lebs' contains number of LEBs to reserve */
	if (min_idx_lebs > c->lst.idx_lebs)
		rsvd_idx_lebs = min_idx_lebs - c->lst.idx_lebs;
	else
		rsvd_idx_lebs = 0;

	/*
	 * The number of LEBs that are available to be used by the index is:
	 *
	 *    @c->lst.empty_lebs + @c->freeable_cnt + @c->idx_gc_cnt -
	 *    @c->lst.taken_empty_lebs
	 *
	 * @c->lst.empty_lebs are available because they are empty.
	 * @c->freeable_cnt are available because they contain only free and
	 * dirty space, @c->idx_gc_cnt are available because they are index
	 * LEBs that have been garbage collected and are awaiting the commit
	 * before they can be used. And the in-the-gaps method will grab these
	 * if it needs them. @c->lst.taken_empty_lebs are empty LEBs that have
	 * already been allocated for some purpose.
	 *
	 * Note, @c->idx_gc_cnt is included to both @c->lst.empty_lebs (because
	 * these LEBs are empty) and to @c->lst.taken_empty_lebs (because they
	 * are taken until after the commit).
	 *
	 * Note, @c->lst.taken_empty_lebs may temporarily be higher by one
	 * because of the way we serialize LEB allocations and budgeting. See a
	 * comment in 'ubifs_find_free_space()'.
	 */
	lebs = c->lst.empty_lebs + c->freeable_cnt + c->idx_gc_cnt -
	       c->lst.taken_empty_lebs;
	if (unlikely(rsvd_idx_lebs > lebs)) {
		dbg_budg("out of indexing space: min_idx_lebs %d (old %d), "
			 "rsvd_idx_lebs %d", min_idx_lebs, c->min_idx_lebs,
			 rsvd_idx_lebs);
		return -ENOSPC;
	}

	available = ubifs_calc_available(c, min_idx_lebs);
	outstanding = c->budg_data_growth + c->budg_dd_growth;

	if (unlikely(available < outstanding)) {
		dbg_budg("out of data space: available %lld, outstanding %lld",
			 available, outstanding);
		return -ENOSPC;
	}

	if (available - outstanding <= c->rp_size && !can_use_rp(c))
		return -ENOSPC;

	c->min_idx_lebs = min_idx_lebs;
	return 0;
}

static int calc_idx_growth(const struct ubifs_info *c,
			   const struct ubifs_budget_req *req)
{
	int znodes;

	znodes = req->new_ino + (req->new_page << UBIFS_BLOCKS_PER_PAGE_SHIFT) +
		 req->new_dent;
	return znodes * c->max_idx_node_sz;
}

static int calc_data_growth(const struct ubifs_info *c,
			    const struct ubifs_budget_req *req)
{
	int data_growth;

	data_growth = req->new_ino  ? c->inode_budget : 0;
	if (req->new_page)
		data_growth += c->page_budget;
	if (req->new_dent)
		data_growth += c->dent_budget;
	data_growth += req->new_ino_d;
	return data_growth;
}

static int calc_dd_growth(const struct ubifs_info *c,
			  const struct ubifs_budget_req *req)
{
	int dd_growth;

	dd_growth = req->dirtied_page ? c->page_budget : 0;

	if (req->dirtied_ino)
		dd_growth += c->inode_budget << (req->dirtied_ino - 1);
	if (req->mod_dent)
		dd_growth += c->dent_budget;
	dd_growth += req->dirtied_ino_d;
	return dd_growth;
}

int ubifs_budget_space(struct ubifs_info *c, struct ubifs_budget_req *req)
{
	int uninitialized_var(cmt_retries), uninitialized_var(wb_retries);
	int err, idx_growth, data_growth, dd_growth, retried = 0;

	ubifs_assert(req->new_page <= 1);
	ubifs_assert(req->dirtied_page <= 1);
	ubifs_assert(req->new_dent <= 1);
	ubifs_assert(req->mod_dent <= 1);
	ubifs_assert(req->new_ino <= 1);
	ubifs_assert(req->new_ino_d <= UBIFS_MAX_INO_DATA);
	ubifs_assert(req->dirtied_ino <= 4);
	ubifs_assert(req->dirtied_ino_d <= UBIFS_MAX_INO_DATA * 4);
	ubifs_assert(!(req->new_ino_d & 7));
	ubifs_assert(!(req->dirtied_ino_d & 7));

	data_growth = calc_data_growth(c, req);
	dd_growth = calc_dd_growth(c, req);
	if (!data_growth && !dd_growth)
		return 0;
	idx_growth = calc_idx_growth(c, req);

again:
	spin_lock(&c->space_lock);
	ubifs_assert(c->budg_idx_growth >= 0);
	ubifs_assert(c->budg_data_growth >= 0);
	ubifs_assert(c->budg_dd_growth >= 0);

	if (unlikely(c->nospace) && (c->nospace_rp || !can_use_rp(c))) {
		dbg_budg("no space");
		spin_unlock(&c->space_lock);
		return -ENOSPC;
	}

	c->budg_idx_growth += idx_growth;
	c->budg_data_growth += data_growth;
	c->budg_dd_growth += dd_growth;

	err = do_budget_space(c);
	if (likely(!err)) {
		req->idx_growth = idx_growth;
		req->data_growth = data_growth;
		req->dd_growth = dd_growth;
		spin_unlock(&c->space_lock);
		return 0;
	}

	/* Restore the old values */
	c->budg_idx_growth -= idx_growth;
	c->budg_data_growth -= data_growth;
	c->budg_dd_growth -= dd_growth;
	spin_unlock(&c->space_lock);

	if (req->fast) {
		dbg_budg("no space for fast budgeting");
		return err;
	}

	err = make_free_space(c);
	cond_resched();
	if (err == -EAGAIN) {
		dbg_budg("try again");
		goto again;
	} else if (err == -ENOSPC) {
		if (!retried) {
			retried = 1;
			dbg_budg("-ENOSPC, but anyway try once again");
			goto again;
		}
		dbg_budg("FS is full, -ENOSPC");
		c->nospace = 1;
		if (can_use_rp(c) || c->rp_size == 0)
			c->nospace_rp = 1;
		smp_wmb();
	} else
		ubifs_err("cannot budget space, error %d", err);
	return err;
}

void ubifs_release_budget(struct ubifs_info *c, struct ubifs_budget_req *req)
{
	ubifs_assert(req->new_page <= 1);
	ubifs_assert(req->dirtied_page <= 1);
	ubifs_assert(req->new_dent <= 1);
	ubifs_assert(req->mod_dent <= 1);
	ubifs_assert(req->new_ino <= 1);
	ubifs_assert(req->new_ino_d <= UBIFS_MAX_INO_DATA);
	ubifs_assert(req->dirtied_ino <= 4);
	ubifs_assert(req->dirtied_ino_d <= UBIFS_MAX_INO_DATA * 4);
	ubifs_assert(!(req->new_ino_d & 7));
	ubifs_assert(!(req->dirtied_ino_d & 7));
	if (!req->recalculate) {
		ubifs_assert(req->idx_growth >= 0);
		ubifs_assert(req->data_growth >= 0);
		ubifs_assert(req->dd_growth >= 0);
	}

	if (req->recalculate) {
		req->data_growth = calc_data_growth(c, req);
		req->dd_growth = calc_dd_growth(c, req);
		req->idx_growth = calc_idx_growth(c, req);
	}

	if (!req->data_growth && !req->dd_growth)
		return;

	c->nospace = c->nospace_rp = 0;
	smp_wmb();

	spin_lock(&c->space_lock);
	c->budg_idx_growth -= req->idx_growth;
	c->budg_uncommitted_idx += req->idx_growth;
	c->budg_data_growth -= req->data_growth;
	c->budg_dd_growth -= req->dd_growth;
	c->min_idx_lebs = ubifs_calc_min_idx_lebs(c);

	ubifs_assert(c->budg_idx_growth >= 0);
	ubifs_assert(c->budg_data_growth >= 0);
	ubifs_assert(c->budg_dd_growth >= 0);
	ubifs_assert(c->min_idx_lebs < c->main_lebs);
	ubifs_assert(!(c->budg_idx_growth & 7));
	ubifs_assert(!(c->budg_data_growth & 7));
	ubifs_assert(!(c->budg_dd_growth & 7));
	spin_unlock(&c->space_lock);
}

void ubifs_convert_page_budget(struct ubifs_info *c)
{
	spin_lock(&c->space_lock);
	/* Release the index growth reservation */
	c->budg_idx_growth -= c->max_idx_node_sz << UBIFS_BLOCKS_PER_PAGE_SHIFT;
	/* Release the data growth reservation */
	c->budg_data_growth -= c->page_budget;
	/* Increase the dirty data growth reservation instead */
	c->budg_dd_growth += c->page_budget;
	/* And re-calculate the indexing space reservation */
	c->min_idx_lebs = ubifs_calc_min_idx_lebs(c);
	spin_unlock(&c->space_lock);
}

void ubifs_release_dirty_inode_budget(struct ubifs_info *c,
				      struct ubifs_inode *ui)
{
	struct ubifs_budget_req req;

	memset(&req, 0, sizeof(struct ubifs_budget_req));
	/* The "no space" flags will be cleared because dd_growth is > 0 */
	req.dd_growth = c->inode_budget + ALIGN(ui->data_len, 8);
	ubifs_release_budget(c, &req);
}

long long ubifs_reported_space(const struct ubifs_info *c, long long free)
{
	int divisor, factor, f;

	/*
	 * Reported space size is @free * X, where X is UBIFS block size
	 * divided by UBIFS block size + all overhead one data block
	 * introduces. The overhead is the node header + indexing overhead.
	 *
	 * Indexing overhead calculations are based on the following formula:
	 * I = N/(f - 1) + 1, where I - number of indexing nodes, N - number
	 * of data nodes, f - fanout. Because effective UBIFS fanout is twice
	 * as less than maximum fanout, we assume that each data node
	 * introduces 3 * @c->max_idx_node_sz / (@c->fanout/2 - 1) bytes.
	 * Note, the multiplier 3 is because UBIFS reserves thrice as more space
	 * for the index.
	 */
	f = c->fanout > 3 ? c->fanout >> 1 : 2;
	factor = UBIFS_BLOCK_SIZE;
	divisor = UBIFS_MAX_DATA_NODE_SZ;
	divisor += (c->max_idx_node_sz * 3) / (f - 1);
	free *= factor;
	return div_u64(free, divisor);
}

long long ubifs_get_free_space_nolock(struct ubifs_info *c)
{
	int rsvd_idx_lebs, lebs;
	long long available, outstanding, free;

	ubifs_assert(c->min_idx_lebs == ubifs_calc_min_idx_lebs(c));
	outstanding = c->budg_data_growth + c->budg_dd_growth;
	available = ubifs_calc_available(c, c->min_idx_lebs);

	/*
	 * When reporting free space to user-space, UBIFS guarantees that it is
	 * possible to write a file of free space size. This means that for
	 * empty LEBs we may use more precise calculations than
	 * 'ubifs_calc_available()' is using. Namely, we know that in empty
	 * LEBs we would waste only @c->leb_overhead bytes, not @c->dark_wm.
	 * Thus, amend the available space.
	 *
	 * Note, the calculations below are similar to what we have in
	 * 'do_budget_space()', so refer there for comments.
	 */
	if (c->min_idx_lebs > c->lst.idx_lebs)
		rsvd_idx_lebs = c->min_idx_lebs - c->lst.idx_lebs;
	else
		rsvd_idx_lebs = 0;
	lebs = c->lst.empty_lebs + c->freeable_cnt + c->idx_gc_cnt -
	       c->lst.taken_empty_lebs;
	lebs -= rsvd_idx_lebs;
	available += lebs * (c->dark_wm - c->leb_overhead);

	if (available > outstanding)
		free = ubifs_reported_space(c, available - outstanding);
	else
		free = 0;
	return free;
}

long long ubifs_get_free_space(struct ubifs_info *c)
{
	long long free;

	spin_lock(&c->space_lock);
	free = ubifs_get_free_space_nolock(c);
	spin_unlock(&c->space_lock);

	return free;
}
