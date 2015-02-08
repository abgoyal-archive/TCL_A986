

#include "debug.h"
#include "dir.h"
#include "endian.h"
#include "malloc.h"
#include "ntfs.h"

static inline void ntfs_rl_mm(runlist_element *base, int dst, int src,
		int size)
{
	if (likely((dst != src) && (size > 0)))
		memmove(base + dst, base + src, size * sizeof(*base));
}

static inline void ntfs_rl_mc(runlist_element *dstbase, int dst,
		runlist_element *srcbase, int src, int size)
{
	if (likely(size > 0))
		memcpy(dstbase + dst, srcbase + src, size * sizeof(*dstbase));
}

static inline runlist_element *ntfs_rl_realloc(runlist_element *rl,
		int old_size, int new_size)
{
	runlist_element *new_rl;

	old_size = PAGE_ALIGN(old_size * sizeof(*rl));
	new_size = PAGE_ALIGN(new_size * sizeof(*rl));
	if (old_size == new_size)
		return rl;

	new_rl = ntfs_malloc_nofs(new_size);
	if (unlikely(!new_rl))
		return ERR_PTR(-ENOMEM);

	if (likely(rl != NULL)) {
		if (unlikely(old_size > new_size))
			old_size = new_size;
		memcpy(new_rl, rl, old_size);
		ntfs_free(rl);
	}
	return new_rl;
}

static inline runlist_element *ntfs_rl_realloc_nofail(runlist_element *rl,
		int old_size, int new_size)
{
	runlist_element *new_rl;

	old_size = PAGE_ALIGN(old_size * sizeof(*rl));
	new_size = PAGE_ALIGN(new_size * sizeof(*rl));
	if (old_size == new_size)
		return rl;

	new_rl = ntfs_malloc_nofs_nofail(new_size);
	BUG_ON(!new_rl);

	if (likely(rl != NULL)) {
		if (unlikely(old_size > new_size))
			old_size = new_size;
		memcpy(new_rl, rl, old_size);
		ntfs_free(rl);
	}
	return new_rl;
}

static inline bool ntfs_are_rl_mergeable(runlist_element *dst,
		runlist_element *src)
{
	BUG_ON(!dst);
	BUG_ON(!src);

	/* We can merge unmapped regions even if they are misaligned. */
	if ((dst->lcn == LCN_RL_NOT_MAPPED) && (src->lcn == LCN_RL_NOT_MAPPED))
		return true;
	/* If the runs are misaligned, we cannot merge them. */
	if ((dst->vcn + dst->length) != src->vcn)
		return false;
	/* If both runs are non-sparse and contiguous, we can merge them. */
	if ((dst->lcn >= 0) && (src->lcn >= 0) &&
			((dst->lcn + dst->length) == src->lcn))
		return true;
	/* If we are merging two holes, we can merge them. */
	if ((dst->lcn == LCN_HOLE) && (src->lcn == LCN_HOLE))
		return true;
	/* Cannot merge. */
	return false;
}

static inline void __ntfs_rl_merge(runlist_element *dst, runlist_element *src)
{
	dst->length += src->length;
}

static inline runlist_element *ntfs_rl_append(runlist_element *dst,
		int dsize, runlist_element *src, int ssize, int loc)
{
	bool right = false;	/* Right end of @src needs merging. */
	int marker;		/* End of the inserted runs. */

	BUG_ON(!dst);
	BUG_ON(!src);

	/* First, check if the right hand end needs merging. */
	if ((loc + 1) < dsize)
		right = ntfs_are_rl_mergeable(src + ssize - 1, dst + loc + 1);

	/* Space required: @dst size + @src size, less one if we merged. */
	dst = ntfs_rl_realloc(dst, dsize, dsize + ssize - right);
	if (IS_ERR(dst))
		return dst;
	/*
	 * We are guaranteed to succeed from here so can start modifying the
	 * original runlists.
	 */

	/* First, merge the right hand end, if necessary. */
	if (right)
		__ntfs_rl_merge(src + ssize - 1, dst + loc + 1);

	/* First run after the @src runs that have been inserted. */
	marker = loc + ssize + 1;

	/* Move the tail of @dst out of the way, then copy in @src. */
	ntfs_rl_mm(dst, marker, loc + 1 + right, dsize - (loc + 1 + right));
	ntfs_rl_mc(dst, loc + 1, src, 0, ssize);

	/* Adjust the size of the preceding hole. */
	dst[loc].length = dst[loc + 1].vcn - dst[loc].vcn;

	/* We may have changed the length of the file, so fix the end marker */
	if (dst[marker].lcn == LCN_ENOENT)
		dst[marker].vcn = dst[marker - 1].vcn + dst[marker - 1].length;

	return dst;
}

static inline runlist_element *ntfs_rl_insert(runlist_element *dst,
		int dsize, runlist_element *src, int ssize, int loc)
{
	bool left = false;	/* Left end of @src needs merging. */
	bool disc = false;	/* Discontinuity between @dst and @src. */
	int marker;		/* End of the inserted runs. */

	BUG_ON(!dst);
	BUG_ON(!src);

	/*
	 * disc => Discontinuity between the end of @dst and the start of @src.
	 *	   This means we might need to insert a "not mapped" run.
	 */
	if (loc == 0)
		disc = (src[0].vcn > 0);
	else {
		s64 merged_length;

		left = ntfs_are_rl_mergeable(dst + loc - 1, src);

		merged_length = dst[loc - 1].length;
		if (left)
			merged_length += src->length;

		disc = (src[0].vcn > dst[loc - 1].vcn + merged_length);
	}
	/*
	 * Space required: @dst size + @src size, less one if we merged, plus
	 * one if there was a discontinuity.
	 */
	dst = ntfs_rl_realloc(dst, dsize, dsize + ssize - left + disc);
	if (IS_ERR(dst))
		return dst;
	/*
	 * We are guaranteed to succeed from here so can start modifying the
	 * original runlist.
	 */
	if (left)
		__ntfs_rl_merge(dst + loc - 1, src);
	/*
	 * First run after the @src runs that have been inserted.
	 * Nominally,  @marker equals @loc + @ssize, i.e. location + number of
	 * runs in @src.  However, if @left, then the first run in @src has
	 * been merged with one in @dst.  And if @disc, then @dst and @src do
	 * not meet and we need an extra run to fill the gap.
	 */
	marker = loc + ssize - left + disc;

	/* Move the tail of @dst out of the way, then copy in @src. */
	ntfs_rl_mm(dst, marker, loc, dsize - loc);
	ntfs_rl_mc(dst, loc + disc, src, left, ssize - left);

	/* Adjust the VCN of the first run after the insertion... */
	dst[marker].vcn = dst[marker - 1].vcn + dst[marker - 1].length;
	/* ... and the length. */
	if (dst[marker].lcn == LCN_HOLE || dst[marker].lcn == LCN_RL_NOT_MAPPED)
		dst[marker].length = dst[marker + 1].vcn - dst[marker].vcn;

	/* Writing beyond the end of the file and there is a discontinuity. */
	if (disc) {
		if (loc > 0) {
			dst[loc].vcn = dst[loc - 1].vcn + dst[loc - 1].length;
			dst[loc].length = dst[loc + 1].vcn - dst[loc].vcn;
		} else {
			dst[loc].vcn = 0;
			dst[loc].length = dst[loc + 1].vcn;
		}
		dst[loc].lcn = LCN_RL_NOT_MAPPED;
	}
	return dst;
}

static inline runlist_element *ntfs_rl_replace(runlist_element *dst,
		int dsize, runlist_element *src, int ssize, int loc)
{
	signed delta;
	bool left = false;	/* Left end of @src needs merging. */
	bool right = false;	/* Right end of @src needs merging. */
	int tail;		/* Start of tail of @dst. */
	int marker;		/* End of the inserted runs. */

	BUG_ON(!dst);
	BUG_ON(!src);

	/* First, see if the left and right ends need merging. */
	if ((loc + 1) < dsize)
		right = ntfs_are_rl_mergeable(src + ssize - 1, dst + loc + 1);
	if (loc > 0)
		left = ntfs_are_rl_mergeable(dst + loc - 1, src);
	/*
	 * Allocate some space.  We will need less if the left, right, or both
	 * ends get merged.  The -1 accounts for the run being replaced.
	 */
	delta = ssize - 1 - left - right;
	if (delta > 0) {
		dst = ntfs_rl_realloc(dst, dsize, dsize + delta);
		if (IS_ERR(dst))
			return dst;
	}
	/*
	 * We are guaranteed to succeed from here so can start modifying the
	 * original runlists.
	 */

	/* First, merge the left and right ends, if necessary. */
	if (right)
		__ntfs_rl_merge(src + ssize - 1, dst + loc + 1);
	if (left)
		__ntfs_rl_merge(dst + loc - 1, src);
	/*
	 * Offset of the tail of @dst.  This needs to be moved out of the way
	 * to make space for the runs to be copied from @src, i.e. the first
	 * run of the tail of @dst.
	 * Nominally, @tail equals @loc + 1, i.e. location, skipping the
	 * replaced run.  However, if @right, then one of @dst's runs is
	 * already merged into @src.
	 */
	tail = loc + right + 1;
	/*
	 * First run after the @src runs that have been inserted, i.e. where
	 * the tail of @dst needs to be moved to.
	 * Nominally, @marker equals @loc + @ssize, i.e. location + number of
	 * runs in @src.  However, if @left, then the first run in @src has
	 * been merged with one in @dst.
	 */
	marker = loc + ssize - left;

	/* Move the tail of @dst out of the way, then copy in @src. */
	ntfs_rl_mm(dst, marker, tail, dsize - tail);
	ntfs_rl_mc(dst, loc, src, left, ssize - left);

	/* We may have changed the length of the file, so fix the end marker. */
	if (dsize - tail > 0 && dst[marker].lcn == LCN_ENOENT)
		dst[marker].vcn = dst[marker - 1].vcn + dst[marker - 1].length;
	return dst;
}

static inline runlist_element *ntfs_rl_split(runlist_element *dst, int dsize,
		runlist_element *src, int ssize, int loc)
{
	BUG_ON(!dst);
	BUG_ON(!src);

	/* Space required: @dst size + @src size + one new hole. */
	dst = ntfs_rl_realloc(dst, dsize, dsize + ssize + 1);
	if (IS_ERR(dst))
		return dst;
	/*
	 * We are guaranteed to succeed from here so can start modifying the
	 * original runlists.
	 */

	/* Move the tail of @dst out of the way, then copy in @src. */
	ntfs_rl_mm(dst, loc + 1 + ssize, loc, dsize - loc);
	ntfs_rl_mc(dst, loc + 1, src, 0, ssize);

	/* Adjust the size of the holes either size of @src. */
	dst[loc].length		= dst[loc+1].vcn       - dst[loc].vcn;
	dst[loc+ssize+1].vcn    = dst[loc+ssize].vcn   + dst[loc+ssize].length;
	dst[loc+ssize+1].length = dst[loc+ssize+2].vcn - dst[loc+ssize+1].vcn;

	return dst;
}

runlist_element *ntfs_runlists_merge(runlist_element *drl,
		runlist_element *srl)
{
	int di, si;		/* Current index into @[ds]rl. */
	int sstart;		/* First index with lcn > LCN_RL_NOT_MAPPED. */
	int dins;		/* Index into @drl at which to insert @srl. */
	int dend, send;		/* Last index into @[ds]rl. */
	int dfinal, sfinal;	/* The last index into @[ds]rl with
				   lcn >= LCN_HOLE. */
	int marker = 0;
	VCN marker_vcn = 0;

#ifdef DEBUG
	ntfs_debug("dst:");
	ntfs_debug_dump_runlist(drl);
	ntfs_debug("src:");
	ntfs_debug_dump_runlist(srl);
#endif

	/* Check for silly calling... */
	if (unlikely(!srl))
		return drl;
	if (IS_ERR(srl) || IS_ERR(drl))
		return ERR_PTR(-EINVAL);

	/* Check for the case where the first mapping is being done now. */
	if (unlikely(!drl)) {
		drl = srl;
		/* Complete the source runlist if necessary. */
		if (unlikely(drl[0].vcn)) {
			/* Scan to the end of the source runlist. */
			for (dend = 0; likely(drl[dend].length); dend++)
				;
			dend++;
			drl = ntfs_rl_realloc(drl, dend, dend + 1);
			if (IS_ERR(drl))
				return drl;
			/* Insert start element at the front of the runlist. */
			ntfs_rl_mm(drl, 1, 0, dend);
			drl[0].vcn = 0;
			drl[0].lcn = LCN_RL_NOT_MAPPED;
			drl[0].length = drl[1].vcn;
		}
		goto finished;
	}

	si = di = 0;

	/* Skip any unmapped start element(s) in the source runlist. */
	while (srl[si].length && srl[si].lcn < LCN_HOLE)
		si++;

	/* Can't have an entirely unmapped source runlist. */
	BUG_ON(!srl[si].length);

	/* Record the starting points. */
	sstart = si;

	/*
	 * Skip forward in @drl until we reach the position where @srl needs to
	 * be inserted. If we reach the end of @drl, @srl just needs to be
	 * appended to @drl.
	 */
	for (; drl[di].length; di++) {
		if (drl[di].vcn + drl[di].length > srl[sstart].vcn)
			break;
	}
	dins = di;

	/* Sanity check for illegal overlaps. */
	if ((drl[di].vcn == srl[si].vcn) && (drl[di].lcn >= 0) &&
			(srl[si].lcn >= 0)) {
		ntfs_error(NULL, "Run lists overlap. Cannot merge!");
		return ERR_PTR(-ERANGE);
	}

	/* Scan to the end of both runlists in order to know their sizes. */
	for (send = si; srl[send].length; send++)
		;
	for (dend = di; drl[dend].length; dend++)
		;

	if (srl[send].lcn == LCN_ENOENT)
		marker_vcn = srl[marker = send].vcn;

	/* Scan to the last element with lcn >= LCN_HOLE. */
	for (sfinal = send; sfinal >= 0 && srl[sfinal].lcn < LCN_HOLE; sfinal--)
		;
	for (dfinal = dend; dfinal >= 0 && drl[dfinal].lcn < LCN_HOLE; dfinal--)
		;

	{
	bool start;
	bool finish;
	int ds = dend + 1;		/* Number of elements in drl & srl */
	int ss = sfinal - sstart + 1;

	start  = ((drl[dins].lcn <  LCN_RL_NOT_MAPPED) ||    /* End of file   */
		  (drl[dins].vcn == srl[sstart].vcn));	     /* Start of hole */
	finish = ((drl[dins].lcn >= LCN_RL_NOT_MAPPED) &&    /* End of file   */
		 ((drl[dins].vcn + drl[dins].length) <=      /* End of hole   */
		  (srl[send - 1].vcn + srl[send - 1].length)));

	/* Or we will lose an end marker. */
	if (finish && !drl[dins].length)
		ss++;
	if (marker && (drl[dins].vcn + drl[dins].length > srl[send - 1].vcn))
		finish = false;
#if 0
	ntfs_debug("dfinal = %i, dend = %i", dfinal, dend);
	ntfs_debug("sstart = %i, sfinal = %i, send = %i", sstart, sfinal, send);
	ntfs_debug("start = %i, finish = %i", start, finish);
	ntfs_debug("ds = %i, ss = %i, dins = %i", ds, ss, dins);
#endif
	if (start) {
		if (finish)
			drl = ntfs_rl_replace(drl, ds, srl + sstart, ss, dins);
		else
			drl = ntfs_rl_insert(drl, ds, srl + sstart, ss, dins);
	} else {
		if (finish)
			drl = ntfs_rl_append(drl, ds, srl + sstart, ss, dins);
		else
			drl = ntfs_rl_split(drl, ds, srl + sstart, ss, dins);
	}
	if (IS_ERR(drl)) {
		ntfs_error(NULL, "Merge failed.");
		return drl;
	}
	ntfs_free(srl);
	if (marker) {
		ntfs_debug("Triggering marker code.");
		for (ds = dend; drl[ds].length; ds++)
			;
		/* We only need to care if @srl ended after @drl. */
		if (drl[ds].vcn <= marker_vcn) {
			int slots = 0;

			if (drl[ds].vcn == marker_vcn) {
				ntfs_debug("Old marker = 0x%llx, replacing "
						"with LCN_ENOENT.",
						(unsigned long long)
						drl[ds].lcn);
				drl[ds].lcn = LCN_ENOENT;
				goto finished;
			}
			/*
			 * We need to create an unmapped runlist element in
			 * @drl or extend an existing one before adding the
			 * ENOENT terminator.
			 */
			if (drl[ds].lcn == LCN_ENOENT) {
				ds--;
				slots = 1;
			}
			if (drl[ds].lcn != LCN_RL_NOT_MAPPED) {
				/* Add an unmapped runlist element. */
				if (!slots) {
					drl = ntfs_rl_realloc_nofail(drl, ds,
							ds + 2);
					slots = 2;
				}
				ds++;
				/* Need to set vcn if it isn't set already. */
				if (slots != 1)
					drl[ds].vcn = drl[ds - 1].vcn +
							drl[ds - 1].length;
				drl[ds].lcn = LCN_RL_NOT_MAPPED;
				/* We now used up a slot. */
				slots--;
			}
			drl[ds].length = marker_vcn - drl[ds].vcn;
			/* Finally add the ENOENT terminator. */
			ds++;
			if (!slots)
				drl = ntfs_rl_realloc_nofail(drl, ds, ds + 1);
			drl[ds].vcn = marker_vcn;
			drl[ds].lcn = LCN_ENOENT;
			drl[ds].length = (s64)0;
		}
	}
	}

finished:
	/* The merge was completed successfully. */
	ntfs_debug("Merged runlist:");
	ntfs_debug_dump_runlist(drl);
	return drl;
}

runlist_element *ntfs_mapping_pairs_decompress(const ntfs_volume *vol,
		const ATTR_RECORD *attr, runlist_element *old_rl)
{
	VCN vcn;		/* Current vcn. */
	LCN lcn;		/* Current lcn. */
	s64 deltaxcn;		/* Change in [vl]cn. */
	runlist_element *rl;	/* The output runlist. */
	u8 *buf;		/* Current position in mapping pairs array. */
	u8 *attr_end;		/* End of attribute. */
	int rlsize;		/* Size of runlist buffer. */
	u16 rlpos;		/* Current runlist position in units of
				   runlist_elements. */
	u8 b;			/* Current byte offset in buf. */

#ifdef DEBUG
	/* Make sure attr exists and is non-resident. */
	if (!attr || !attr->non_resident || sle64_to_cpu(
			attr->data.non_resident.lowest_vcn) < (VCN)0) {
		ntfs_error(vol->sb, "Invalid arguments.");
		return ERR_PTR(-EINVAL);
	}
#endif
	/* Start at vcn = lowest_vcn and lcn 0. */
	vcn = sle64_to_cpu(attr->data.non_resident.lowest_vcn);
	lcn = 0;
	/* Get start of the mapping pairs array. */
	buf = (u8*)attr + le16_to_cpu(
			attr->data.non_resident.mapping_pairs_offset);
	attr_end = (u8*)attr + le32_to_cpu(attr->length);
	if (unlikely(buf < (u8*)attr || buf > attr_end)) {
		ntfs_error(vol->sb, "Corrupt attribute.");
		return ERR_PTR(-EIO);
	}
	/* If the mapping pairs array is valid but empty, nothing to do. */
	if (!vcn && !*buf)
		return old_rl;
	/* Current position in runlist array. */
	rlpos = 0;
	/* Allocate first page and set current runlist size to one page. */
	rl = ntfs_malloc_nofs(rlsize = PAGE_SIZE);
	if (unlikely(!rl))
		return ERR_PTR(-ENOMEM);
	/* Insert unmapped starting element if necessary. */
	if (vcn) {
		rl->vcn = 0;
		rl->lcn = LCN_RL_NOT_MAPPED;
		rl->length = vcn;
		rlpos++;
	}
	while (buf < attr_end && *buf) {
		/*
		 * Allocate more memory if needed, including space for the
		 * not-mapped and terminator elements. ntfs_malloc_nofs()
		 * operates on whole pages only.
		 */
		if (((rlpos + 3) * sizeof(*old_rl)) > rlsize) {
			runlist_element *rl2;

			rl2 = ntfs_malloc_nofs(rlsize + (int)PAGE_SIZE);
			if (unlikely(!rl2)) {
				ntfs_free(rl);
				return ERR_PTR(-ENOMEM);
			}
			memcpy(rl2, rl, rlsize);
			ntfs_free(rl);
			rl = rl2;
			rlsize += PAGE_SIZE;
		}
		/* Enter the current vcn into the current runlist element. */
		rl[rlpos].vcn = vcn;
		/*
		 * Get the change in vcn, i.e. the run length in clusters.
		 * Doing it this way ensures that we signextend negative values.
		 * A negative run length doesn't make any sense, but hey, I
		 * didn't make up the NTFS specs and Windows NT4 treats the run
		 * length as a signed value so that's how it is...
		 */
		b = *buf & 0xf;
		if (b) {
			if (unlikely(buf + b > attr_end))
				goto io_error;
			for (deltaxcn = (s8)buf[b--]; b; b--)
				deltaxcn = (deltaxcn << 8) + buf[b];
		} else { /* The length entry is compulsory. */
			ntfs_error(vol->sb, "Missing length entry in mapping "
					"pairs array.");
			deltaxcn = (s64)-1;
		}
		/*
		 * Assume a negative length to indicate data corruption and
		 * hence clean-up and return NULL.
		 */
		if (unlikely(deltaxcn < 0)) {
			ntfs_error(vol->sb, "Invalid length in mapping pairs "
					"array.");
			goto err_out;
		}
		/*
		 * Enter the current run length into the current runlist
		 * element.
		 */
		rl[rlpos].length = deltaxcn;
		/* Increment the current vcn by the current run length. */
		vcn += deltaxcn;
		/*
		 * There might be no lcn change at all, as is the case for
		 * sparse clusters on NTFS 3.0+, in which case we set the lcn
		 * to LCN_HOLE.
		 */
		if (!(*buf & 0xf0))
			rl[rlpos].lcn = LCN_HOLE;
		else {
			/* Get the lcn change which really can be negative. */
			u8 b2 = *buf & 0xf;
			b = b2 + ((*buf >> 4) & 0xf);
			if (buf + b > attr_end)
				goto io_error;
			for (deltaxcn = (s8)buf[b--]; b > b2; b--)
				deltaxcn = (deltaxcn << 8) + buf[b];
			/* Change the current lcn to its new value. */
			lcn += deltaxcn;
#ifdef DEBUG
			/*
			 * On NTFS 1.2-, apparently can have lcn == -1 to
			 * indicate a hole. But we haven't verified ourselves
			 * whether it is really the lcn or the deltaxcn that is
			 * -1. So if either is found give us a message so we
			 * can investigate it further!
			 */
			if (vol->major_ver < 3) {
				if (unlikely(deltaxcn == (LCN)-1))
					ntfs_error(vol->sb, "lcn delta == -1");
				if (unlikely(lcn == (LCN)-1))
					ntfs_error(vol->sb, "lcn == -1");
			}
#endif
			/* Check lcn is not below -1. */
			if (unlikely(lcn < (LCN)-1)) {
				ntfs_error(vol->sb, "Invalid LCN < -1 in "
						"mapping pairs array.");
				goto err_out;
			}
			/* Enter the current lcn into the runlist element. */
			rl[rlpos].lcn = lcn;
		}
		/* Get to the next runlist element. */
		rlpos++;
		/* Increment the buffer position to the next mapping pair. */
		buf += (*buf & 0xf) + ((*buf >> 4) & 0xf) + 1;
	}
	if (unlikely(buf >= attr_end))
		goto io_error;
	/*
	 * If there is a highest_vcn specified, it must be equal to the final
	 * vcn in the runlist - 1, or something has gone badly wrong.
	 */
	deltaxcn = sle64_to_cpu(attr->data.non_resident.highest_vcn);
	if (unlikely(deltaxcn && vcn - 1 != deltaxcn)) {
mpa_err:
		ntfs_error(vol->sb, "Corrupt mapping pairs array in "
				"non-resident attribute.");
		goto err_out;
	}
	/* Setup not mapped runlist element if this is the base extent. */
	if (!attr->data.non_resident.lowest_vcn) {
		VCN max_cluster;

		max_cluster = ((sle64_to_cpu(
				attr->data.non_resident.allocated_size) +
				vol->cluster_size - 1) >>
				vol->cluster_size_bits) - 1;
		/*
		 * A highest_vcn of zero means this is a single extent
		 * attribute so simply terminate the runlist with LCN_ENOENT).
		 */
		if (deltaxcn) {
			/*
			 * If there is a difference between the highest_vcn and
			 * the highest cluster, the runlist is either corrupt
			 * or, more likely, there are more extents following
			 * this one.
			 */
			if (deltaxcn < max_cluster) {
				ntfs_debug("More extents to follow; deltaxcn "
						"= 0x%llx, max_cluster = "
						"0x%llx",
						(unsigned long long)deltaxcn,
						(unsigned long long)
						max_cluster);
				rl[rlpos].vcn = vcn;
				vcn += rl[rlpos].length = max_cluster -
						deltaxcn;
				rl[rlpos].lcn = LCN_RL_NOT_MAPPED;
				rlpos++;
			} else if (unlikely(deltaxcn > max_cluster)) {
				ntfs_error(vol->sb, "Corrupt attribute.  "
						"deltaxcn = 0x%llx, "
						"max_cluster = 0x%llx",
						(unsigned long long)deltaxcn,
						(unsigned long long)
						max_cluster);
				goto mpa_err;
			}
		}
		rl[rlpos].lcn = LCN_ENOENT;
	} else /* Not the base extent. There may be more extents to follow. */
		rl[rlpos].lcn = LCN_RL_NOT_MAPPED;

	/* Setup terminating runlist element. */
	rl[rlpos].vcn = vcn;
	rl[rlpos].length = (s64)0;
	/* If no existing runlist was specified, we are done. */
	if (!old_rl) {
		ntfs_debug("Mapping pairs array successfully decompressed:");
		ntfs_debug_dump_runlist(rl);
		return rl;
	}
	/* Now combine the new and old runlists checking for overlaps. */
	old_rl = ntfs_runlists_merge(old_rl, rl);
	if (likely(!IS_ERR(old_rl)))
		return old_rl;
	ntfs_free(rl);
	ntfs_error(vol->sb, "Failed to merge runlists.");
	return old_rl;
io_error:
	ntfs_error(vol->sb, "Corrupt attribute.");
err_out:
	ntfs_free(rl);
	return ERR_PTR(-EIO);
}

LCN ntfs_rl_vcn_to_lcn(const runlist_element *rl, const VCN vcn)
{
	int i;

	BUG_ON(vcn < 0);
	/*
	 * If rl is NULL, assume that we have found an unmapped runlist. The
	 * caller can then attempt to map it and fail appropriately if
	 * necessary.
	 */
	if (unlikely(!rl))
		return LCN_RL_NOT_MAPPED;

	/* Catch out of lower bounds vcn. */
	if (unlikely(vcn < rl[0].vcn))
		return LCN_ENOENT;

	for (i = 0; likely(rl[i].length); i++) {
		if (unlikely(vcn < rl[i+1].vcn)) {
			if (likely(rl[i].lcn >= (LCN)0))
				return rl[i].lcn + (vcn - rl[i].vcn);
			return rl[i].lcn;
		}
	}
	/*
	 * The terminator element is setup to the correct value, i.e. one of
	 * LCN_HOLE, LCN_RL_NOT_MAPPED, or LCN_ENOENT.
	 */
	if (likely(rl[i].lcn < (LCN)0))
		return rl[i].lcn;
	/* Just in case... We could replace this with BUG() some day. */
	return LCN_ENOENT;
}

#ifdef NTFS_RW

runlist_element *ntfs_rl_find_vcn_nolock(runlist_element *rl, const VCN vcn)
{
	BUG_ON(vcn < 0);
	if (unlikely(!rl || vcn < rl[0].vcn))
		return NULL;
	while (likely(rl->length)) {
		if (unlikely(vcn < rl[1].vcn)) {
			if (likely(rl->lcn >= LCN_HOLE))
				return rl;
			return NULL;
		}
		rl++;
	}
	if (likely(rl->lcn == LCN_ENOENT))
		return rl;
	return NULL;
}

static inline int ntfs_get_nr_significant_bytes(const s64 n)
{
	s64 l = n;
	int i;
	s8 j;

	i = 0;
	do {
		l >>= 8;
		i++;
	} while (l != 0 && l != -1);
	j = (n >> 8 * (i - 1)) & 0xff;
	/* If the sign bit is wrong, we need an extra byte. */
	if ((n < 0 && j >= 0) || (n > 0 && j < 0))
		i++;
	return i;
}

int ntfs_get_size_for_mapping_pairs(const ntfs_volume *vol,
		const runlist_element *rl, const VCN first_vcn,
		const VCN last_vcn)
{
	LCN prev_lcn;
	int rls;
	bool the_end = false;

	BUG_ON(first_vcn < 0);
	BUG_ON(last_vcn < -1);
	BUG_ON(last_vcn >= 0 && first_vcn > last_vcn);
	if (!rl) {
		BUG_ON(first_vcn);
		BUG_ON(last_vcn > 0);
		return 1;
	}
	/* Skip to runlist element containing @first_vcn. */
	while (rl->length && first_vcn >= rl[1].vcn)
		rl++;
	if (unlikely((!rl->length && first_vcn > rl->vcn) ||
			first_vcn < rl->vcn))
		return -EINVAL;
	prev_lcn = 0;
	/* Always need the termining zero byte. */
	rls = 1;
	/* Do the first partial run if present. */
	if (first_vcn > rl->vcn) {
		s64 delta, length = rl->length;

		/* We know rl->length != 0 already. */
		if (unlikely(length < 0 || rl->lcn < LCN_HOLE))
			goto err_out;
		/*
		 * If @stop_vcn is given and finishes inside this run, cap the
		 * run length.
		 */
		if (unlikely(last_vcn >= 0 && rl[1].vcn > last_vcn)) {
			s64 s1 = last_vcn + 1;
			if (unlikely(rl[1].vcn > s1))
				length = s1 - rl->vcn;
			the_end = true;
		}
		delta = first_vcn - rl->vcn;
		/* Header byte + length. */
		rls += 1 + ntfs_get_nr_significant_bytes(length - delta);
		/*
		 * If the logical cluster number (lcn) denotes a hole and we
		 * are on NTFS 3.0+, we don't store it at all, i.e. we need
		 * zero space.  On earlier NTFS versions we just store the lcn.
		 * Note: this assumes that on NTFS 1.2-, holes are stored with
		 * an lcn of -1 and not a delta_lcn of -1 (unless both are -1).
		 */
		if (likely(rl->lcn >= 0 || vol->major_ver < 3)) {
			prev_lcn = rl->lcn;
			if (likely(rl->lcn >= 0))
				prev_lcn += delta;
			/* Change in lcn. */
			rls += ntfs_get_nr_significant_bytes(prev_lcn);
		}
		/* Go to next runlist element. */
		rl++;
	}
	/* Do the full runs. */
	for (; rl->length && !the_end; rl++) {
		s64 length = rl->length;

		if (unlikely(length < 0 || rl->lcn < LCN_HOLE))
			goto err_out;
		/*
		 * If @stop_vcn is given and finishes inside this run, cap the
		 * run length.
		 */
		if (unlikely(last_vcn >= 0 && rl[1].vcn > last_vcn)) {
			s64 s1 = last_vcn + 1;
			if (unlikely(rl[1].vcn > s1))
				length = s1 - rl->vcn;
			the_end = true;
		}
		/* Header byte + length. */
		rls += 1 + ntfs_get_nr_significant_bytes(length);
		/*
		 * If the logical cluster number (lcn) denotes a hole and we
		 * are on NTFS 3.0+, we don't store it at all, i.e. we need
		 * zero space.  On earlier NTFS versions we just store the lcn.
		 * Note: this assumes that on NTFS 1.2-, holes are stored with
		 * an lcn of -1 and not a delta_lcn of -1 (unless both are -1).
		 */
		if (likely(rl->lcn >= 0 || vol->major_ver < 3)) {
			/* Change in lcn. */
			rls += ntfs_get_nr_significant_bytes(rl->lcn -
					prev_lcn);
			prev_lcn = rl->lcn;
		}
	}
	return rls;
err_out:
	if (rl->lcn == LCN_RL_NOT_MAPPED)
		rls = -EINVAL;
	else
		rls = -EIO;
	return rls;
}

static inline int ntfs_write_significant_bytes(s8 *dst, const s8 *dst_max,
		const s64 n)
{
	s64 l = n;
	int i;
	s8 j;

	i = 0;
	do {
		if (unlikely(dst > dst_max))
			goto err_out;
		*dst++ = l & 0xffll;
		l >>= 8;
		i++;
	} while (l != 0 && l != -1);
	j = (n >> 8 * (i - 1)) & 0xff;
	/* If the sign bit is wrong, we need an extra byte. */
	if (n < 0 && j >= 0) {
		if (unlikely(dst > dst_max))
			goto err_out;
		i++;
		*dst = (s8)-1;
	} else if (n > 0 && j < 0) {
		if (unlikely(dst > dst_max))
			goto err_out;
		i++;
		*dst = (s8)0;
	}
	return i;
err_out:
	return -ENOSPC;
}

int ntfs_mapping_pairs_build(const ntfs_volume *vol, s8 *dst,
		const int dst_len, const runlist_element *rl,
		const VCN first_vcn, const VCN last_vcn, VCN *const stop_vcn)
{
	LCN prev_lcn;
	s8 *dst_max, *dst_next;
	int err = -ENOSPC;
	bool the_end = false;
	s8 len_len, lcn_len;

	BUG_ON(first_vcn < 0);
	BUG_ON(last_vcn < -1);
	BUG_ON(last_vcn >= 0 && first_vcn > last_vcn);
	BUG_ON(dst_len < 1);
	if (!rl) {
		BUG_ON(first_vcn);
		BUG_ON(last_vcn > 0);
		if (stop_vcn)
			*stop_vcn = 0;
		/* Terminator byte. */
		*dst = 0;
		return 0;
	}
	/* Skip to runlist element containing @first_vcn. */
	while (rl->length && first_vcn >= rl[1].vcn)
		rl++;
	if (unlikely((!rl->length && first_vcn > rl->vcn) ||
			first_vcn < rl->vcn))
		return -EINVAL;
	/*
	 * @dst_max is used for bounds checking in
	 * ntfs_write_significant_bytes().
	 */
	dst_max = dst + dst_len - 1;
	prev_lcn = 0;
	/* Do the first partial run if present. */
	if (first_vcn > rl->vcn) {
		s64 delta, length = rl->length;

		/* We know rl->length != 0 already. */
		if (unlikely(length < 0 || rl->lcn < LCN_HOLE))
			goto err_out;
		/*
		 * If @stop_vcn is given and finishes inside this run, cap the
		 * run length.
		 */
		if (unlikely(last_vcn >= 0 && rl[1].vcn > last_vcn)) {
			s64 s1 = last_vcn + 1;
			if (unlikely(rl[1].vcn > s1))
				length = s1 - rl->vcn;
			the_end = true;
		}
		delta = first_vcn - rl->vcn;
		/* Write length. */
		len_len = ntfs_write_significant_bytes(dst + 1, dst_max,
				length - delta);
		if (unlikely(len_len < 0))
			goto size_err;
		/*
		 * If the logical cluster number (lcn) denotes a hole and we
		 * are on NTFS 3.0+, we don't store it at all, i.e. we need
		 * zero space.  On earlier NTFS versions we just write the lcn
		 * change.  FIXME: Do we need to write the lcn change or just
		 * the lcn in that case?  Not sure as I have never seen this
		 * case on NT4. - We assume that we just need to write the lcn
		 * change until someone tells us otherwise... (AIA)
		 */
		if (likely(rl->lcn >= 0 || vol->major_ver < 3)) {
			prev_lcn = rl->lcn;
			if (likely(rl->lcn >= 0))
				prev_lcn += delta;
			/* Write change in lcn. */
			lcn_len = ntfs_write_significant_bytes(dst + 1 +
					len_len, dst_max, prev_lcn);
			if (unlikely(lcn_len < 0))
				goto size_err;
		} else
			lcn_len = 0;
		dst_next = dst + len_len + lcn_len + 1;
		if (unlikely(dst_next > dst_max))
			goto size_err;
		/* Update header byte. */
		*dst = lcn_len << 4 | len_len;
		/* Position at next mapping pairs array element. */
		dst = dst_next;
		/* Go to next runlist element. */
		rl++;
	}
	/* Do the full runs. */
	for (; rl->length && !the_end; rl++) {
		s64 length = rl->length;

		if (unlikely(length < 0 || rl->lcn < LCN_HOLE))
			goto err_out;
		/*
		 * If @stop_vcn is given and finishes inside this run, cap the
		 * run length.
		 */
		if (unlikely(last_vcn >= 0 && rl[1].vcn > last_vcn)) {
			s64 s1 = last_vcn + 1;
			if (unlikely(rl[1].vcn > s1))
				length = s1 - rl->vcn;
			the_end = true;
		}
		/* Write length. */
		len_len = ntfs_write_significant_bytes(dst + 1, dst_max,
				length);
		if (unlikely(len_len < 0))
			goto size_err;
		/*
		 * If the logical cluster number (lcn) denotes a hole and we
		 * are on NTFS 3.0+, we don't store it at all, i.e. we need
		 * zero space.  On earlier NTFS versions we just write the lcn
		 * change.  FIXME: Do we need to write the lcn change or just
		 * the lcn in that case?  Not sure as I have never seen this
		 * case on NT4. - We assume that we just need to write the lcn
		 * change until someone tells us otherwise... (AIA)
		 */
		if (likely(rl->lcn >= 0 || vol->major_ver < 3)) {
			/* Write change in lcn. */
			lcn_len = ntfs_write_significant_bytes(dst + 1 +
					len_len, dst_max, rl->lcn - prev_lcn);
			if (unlikely(lcn_len < 0))
				goto size_err;
			prev_lcn = rl->lcn;
		} else
			lcn_len = 0;
		dst_next = dst + len_len + lcn_len + 1;
		if (unlikely(dst_next > dst_max))
			goto size_err;
		/* Update header byte. */
		*dst = lcn_len << 4 | len_len;
		/* Position at next mapping pairs array element. */
		dst = dst_next;
	}
	/* Success. */
	err = 0;
size_err:
	/* Set stop vcn. */
	if (stop_vcn)
		*stop_vcn = rl->vcn;
	/* Add terminator byte. */
	*dst = 0;
	return err;
err_out:
	if (rl->lcn == LCN_RL_NOT_MAPPED)
		err = -EINVAL;
	else
		err = -EIO;
	return err;
}

int ntfs_rl_truncate_nolock(const ntfs_volume *vol, runlist *const runlist,
		const s64 new_length)
{
	runlist_element *rl;
	int old_size;

	ntfs_debug("Entering for new_length 0x%llx.", (long long)new_length);
	BUG_ON(!runlist);
	BUG_ON(new_length < 0);
	rl = runlist->rl;
	if (!new_length) {
		ntfs_debug("Freeing runlist.");
		runlist->rl = NULL;
		if (rl)
			ntfs_free(rl);
		return 0;
	}
	if (unlikely(!rl)) {
		/*
		 * Create a runlist consisting of a sparse runlist element of
		 * length @new_length followed by a terminator runlist element.
		 */
		rl = ntfs_malloc_nofs(PAGE_SIZE);
		if (unlikely(!rl)) {
			ntfs_error(vol->sb, "Not enough memory to allocate "
					"runlist element buffer.");
			return -ENOMEM;
		}
		runlist->rl = rl;
		rl[1].length = rl->vcn = 0;
		rl->lcn = LCN_HOLE;
		rl[1].vcn = rl->length = new_length;
		rl[1].lcn = LCN_ENOENT;
		return 0;
	}
	BUG_ON(new_length < rl->vcn);
	/* Find @new_length in the runlist. */
	while (likely(rl->length && new_length >= rl[1].vcn))
		rl++;
	/*
	 * If not at the end of the runlist we need to shrink it.
	 * If at the end of the runlist we need to expand it.
	 */
	if (rl->length) {
		runlist_element *trl;
		bool is_end;

		ntfs_debug("Shrinking runlist.");
		/* Determine the runlist size. */
		trl = rl + 1;
		while (likely(trl->length))
			trl++;
		old_size = trl - runlist->rl + 1;
		/* Truncate the run. */
		rl->length = new_length - rl->vcn;
		/*
		 * If a run was partially truncated, make the following runlist
		 * element a terminator.
		 */
		is_end = false;
		if (rl->length) {
			rl++;
			if (!rl->length)
				is_end = true;
			rl->vcn = new_length;
			rl->length = 0;
		}
		rl->lcn = LCN_ENOENT;
		/* Reallocate memory if necessary. */
		if (!is_end) {
			int new_size = rl - runlist->rl + 1;
			rl = ntfs_rl_realloc(runlist->rl, old_size, new_size);
			if (IS_ERR(rl))
				ntfs_warning(vol->sb, "Failed to shrink "
						"runlist buffer.  This just "
						"wastes a bit of memory "
						"temporarily so we ignore it "
						"and return success.");
			else
				runlist->rl = rl;
		}
	} else if (likely(/* !rl->length && */ new_length > rl->vcn)) {
		ntfs_debug("Expanding runlist.");
		/*
		 * If there is a previous runlist element and it is a sparse
		 * one, extend it.  Otherwise need to add a new, sparse runlist
		 * element.
		 */
		if ((rl > runlist->rl) && ((rl - 1)->lcn == LCN_HOLE))
			(rl - 1)->length = new_length - (rl - 1)->vcn;
		else {
			/* Determine the runlist size. */
			old_size = rl - runlist->rl + 1;
			/* Reallocate memory if necessary. */
			rl = ntfs_rl_realloc(runlist->rl, old_size,
					old_size + 1);
			if (IS_ERR(rl)) {
				ntfs_error(vol->sb, "Failed to expand runlist "
						"buffer, aborting.");
				return PTR_ERR(rl);
			}
			runlist->rl = rl;
			/*
			 * Set @rl to the same runlist element in the new
			 * runlist as before in the old runlist.
			 */
			rl += old_size - 1;
			/* Add a new, sparse runlist element. */
			rl->lcn = LCN_HOLE;
			rl->length = new_length - rl->vcn;
			/* Add a new terminator runlist element. */
			rl++;
			rl->length = 0;
		}
		rl->vcn = new_length;
		rl->lcn = LCN_ENOENT;
	} else /* if (unlikely(!rl->length && new_length == rl->vcn)) */ {
		/* Runlist already has same size as requested. */
		rl->lcn = LCN_ENOENT;
	}
	ntfs_debug("Done.");
	return 0;
}

int ntfs_rl_punch_nolock(const ntfs_volume *vol, runlist *const runlist,
		const VCN start, const s64 length)
{
	const VCN end = start + length;
	s64 delta;
	runlist_element *rl, *rl_end, *rl_real_end, *trl;
	int old_size;
	bool lcn_fixup = false;

	ntfs_debug("Entering for start 0x%llx, length 0x%llx.",
			(long long)start, (long long)length);
	BUG_ON(!runlist);
	BUG_ON(start < 0);
	BUG_ON(length < 0);
	BUG_ON(end < 0);
	rl = runlist->rl;
	if (unlikely(!rl)) {
		if (likely(!start && !length))
			return 0;
		return -ENOENT;
	}
	/* Find @start in the runlist. */
	while (likely(rl->length && start >= rl[1].vcn))
		rl++;
	rl_end = rl;
	/* Find @end in the runlist. */
	while (likely(rl_end->length && end >= rl_end[1].vcn)) {
		/* Verify there are no unmapped or error elements. */
		if (unlikely(rl_end->lcn < LCN_HOLE))
			return -EINVAL;
		rl_end++;
	}
	/* Check the last element. */
	if (unlikely(rl_end->length && rl_end->lcn < LCN_HOLE))
		return -EINVAL;
	/* This covers @start being out of bounds, too. */
	if (!rl_end->length && end > rl_end->vcn)
		return -ENOENT;
	if (!length)
		return 0;
	if (!rl->length)
		return -ENOENT;
	rl_real_end = rl_end;
	/* Determine the runlist size. */
	while (likely(rl_real_end->length))
		rl_real_end++;
	old_size = rl_real_end - runlist->rl + 1;
	/* If @start is in a hole simply extend the hole. */
	if (rl->lcn == LCN_HOLE) {
		/*
		 * If both @start and @end are in the same sparse run, we are
		 * done.
		 */
		if (end <= rl[1].vcn) {
			ntfs_debug("Done (requested hole is already sparse).");
			return 0;
		}
extend_hole:
		/* Extend the hole. */
		rl->length = end - rl->vcn;
		/* If @end is in a hole, merge it with the current one. */
		if (rl_end->lcn == LCN_HOLE) {
			rl_end++;
			rl->length = rl_end->vcn - rl->vcn;
		}
		/* We have done the hole.  Now deal with the remaining tail. */
		rl++;
		/* Cut out all runlist elements up to @end. */
		if (rl < rl_end)
			memmove(rl, rl_end, (rl_real_end - rl_end + 1) *
					sizeof(*rl));
		/* Adjust the beginning of the tail if necessary. */
		if (end > rl->vcn) {
			delta = end - rl->vcn;
			rl->vcn = end;
			rl->length -= delta;
			/* Only adjust the lcn if it is real. */
			if (rl->lcn >= 0)
				rl->lcn += delta;
		}
shrink_allocation:
		/* Reallocate memory if the allocation changed. */
		if (rl < rl_end) {
			rl = ntfs_rl_realloc(runlist->rl, old_size,
					old_size - (rl_end - rl));
			if (IS_ERR(rl))
				ntfs_warning(vol->sb, "Failed to shrink "
						"runlist buffer.  This just "
						"wastes a bit of memory "
						"temporarily so we ignore it "
						"and return success.");
			else
				runlist->rl = rl;
		}
		ntfs_debug("Done (extend hole).");
		return 0;
	}
	/*
	 * If @start is at the beginning of a run things are easier as there is
	 * no need to split the first run.
	 */
	if (start == rl->vcn) {
		/*
		 * @start is at the beginning of a run.
		 *
		 * If the previous run is sparse, extend its hole.
		 *
		 * If @end is not in the same run, switch the run to be sparse
		 * and extend the newly created hole.
		 *
		 * Thus both of these cases reduce the problem to the above
		 * case of "@start is in a hole".
		 */
		if (rl > runlist->rl && (rl - 1)->lcn == LCN_HOLE) {
			rl--;
			goto extend_hole;
		}
		if (end >= rl[1].vcn) {
			rl->lcn = LCN_HOLE;
			goto extend_hole;
		}
		/*
		 * The final case is when @end is in the same run as @start.
		 * For this need to split the run into two.  One run for the
		 * sparse region between the beginning of the old run, i.e.
		 * @start, and @end and one for the remaining non-sparse
		 * region, i.e. between @end and the end of the old run.
		 */
		trl = ntfs_rl_realloc(runlist->rl, old_size, old_size + 1);
		if (IS_ERR(trl))
			goto enomem_out;
		old_size++;
		if (runlist->rl != trl) {
			rl = trl + (rl - runlist->rl);
			rl_end = trl + (rl_end - runlist->rl);
			rl_real_end = trl + (rl_real_end - runlist->rl);
			runlist->rl = trl;
		}
split_end:
		/* Shift all the runs up by one. */
		memmove(rl + 1, rl, (rl_real_end - rl + 1) * sizeof(*rl));
		/* Finally, setup the two split runs. */
		rl->lcn = LCN_HOLE;
		rl->length = length;
		rl++;
		rl->vcn += length;
		/* Only adjust the lcn if it is real. */
		if (rl->lcn >= 0 || lcn_fixup)
			rl->lcn += length;
		rl->length -= length;
		ntfs_debug("Done (split one).");
		return 0;
	}
	/*
	 * @start is neither in a hole nor at the beginning of a run.
	 *
	 * If @end is in a hole, things are easier as simply truncating the run
	 * @start is in to end at @start - 1, deleting all runs after that up
	 * to @end, and finally extending the beginning of the run @end is in
	 * to be @start is all that is needed.
	 */
	if (rl_end->lcn == LCN_HOLE) {
		/* Truncate the run containing @start. */
		rl->length = start - rl->vcn;
		rl++;
		/* Cut out all runlist elements up to @end. */
		if (rl < rl_end)
			memmove(rl, rl_end, (rl_real_end - rl_end + 1) *
					sizeof(*rl));
		/* Extend the beginning of the run @end is in to be @start. */
		rl->vcn = start;
		rl->length = rl[1].vcn - start;
		goto shrink_allocation;
	}
	/* 
	 * If @end is not in a hole there are still two cases to distinguish.
	 * Either @end is or is not in the same run as @start.
	 *
	 * The second case is easier as it can be reduced to an already solved
	 * problem by truncating the run @start is in to end at @start - 1.
	 * Then, if @end is in the next run need to split the run into a sparse
	 * run followed by a non-sparse run (already covered above) and if @end
	 * is not in the next run switching it to be sparse, again reduces the
	 * problem to the already covered case of "@start is in a hole".
	 */
	if (end >= rl[1].vcn) {
		/*
		 * If @end is not in the next run, reduce the problem to the
		 * case of "@start is in a hole".
		 */
		if (rl[1].length && end >= rl[2].vcn) {
			/* Truncate the run containing @start. */
			rl->length = start - rl->vcn;
			rl++;
			rl->vcn = start;
			rl->lcn = LCN_HOLE;
			goto extend_hole;
		}
		trl = ntfs_rl_realloc(runlist->rl, old_size, old_size + 1);
		if (IS_ERR(trl))
			goto enomem_out;
		old_size++;
		if (runlist->rl != trl) {
			rl = trl + (rl - runlist->rl);
			rl_end = trl + (rl_end - runlist->rl);
			rl_real_end = trl + (rl_real_end - runlist->rl);
			runlist->rl = trl;
		}
		/* Truncate the run containing @start. */
		rl->length = start - rl->vcn;
		rl++;
		/*
		 * @end is in the next run, reduce the problem to the case
		 * where "@start is at the beginning of a run and @end is in
		 * the same run as @start".
		 */
		delta = rl->vcn - start;
		rl->vcn = start;
		if (rl->lcn >= 0) {
			rl->lcn -= delta;
			/* Need this in case the lcn just became negative. */
			lcn_fixup = true;
		}
		rl->length += delta;
		goto split_end;
	}
	/*
	 * The first case from above, i.e. @end is in the same run as @start.
	 * We need to split the run into three.  One run for the non-sparse
	 * region between the beginning of the old run and @start, one for the
	 * sparse region between @start and @end, and one for the remaining
	 * non-sparse region, i.e. between @end and the end of the old run.
	 */
	trl = ntfs_rl_realloc(runlist->rl, old_size, old_size + 2);
	if (IS_ERR(trl))
		goto enomem_out;
	old_size += 2;
	if (runlist->rl != trl) {
		rl = trl + (rl - runlist->rl);
		rl_end = trl + (rl_end - runlist->rl);
		rl_real_end = trl + (rl_real_end - runlist->rl);
		runlist->rl = trl;
	}
	/* Shift all the runs up by two. */
	memmove(rl + 2, rl, (rl_real_end - rl + 1) * sizeof(*rl));
	/* Finally, setup the three split runs. */
	rl->length = start - rl->vcn;
	rl++;
	rl->vcn = start;
	rl->lcn = LCN_HOLE;
	rl->length = length;
	rl++;
	delta = end - rl->vcn;
	rl->vcn = end;
	rl->lcn += delta;
	rl->length -= delta;
	ntfs_debug("Done (split both).");
	return 0;
enomem_out:
	ntfs_error(vol->sb, "Not enough memory to extend runlist buffer.");
	return -ENOMEM;
}

#endif /* NTFS_RW */
