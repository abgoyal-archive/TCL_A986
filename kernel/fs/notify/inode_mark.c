


#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/writeback.h> /* for inode_lock */

#include <asm/atomic.h>

#include <linux/fsnotify_backend.h>
#include "fsnotify.h"

void fsnotify_get_mark(struct fsnotify_mark_entry *entry)
{
	atomic_inc(&entry->refcnt);
}

void fsnotify_put_mark(struct fsnotify_mark_entry *entry)
{
	if (atomic_dec_and_test(&entry->refcnt))
		entry->free_mark(entry);
}

static void fsnotify_recalc_inode_mask_locked(struct inode *inode)
{
	struct fsnotify_mark_entry *entry;
	struct hlist_node *pos;
	__u32 new_mask = 0;

	assert_spin_locked(&inode->i_lock);

	hlist_for_each_entry(entry, pos, &inode->i_fsnotify_mark_entries, i_list)
		new_mask |= entry->mask;
	inode->i_fsnotify_mask = new_mask;
}

void fsnotify_recalc_inode_mask(struct inode *inode)
{
	spin_lock(&inode->i_lock);
	fsnotify_recalc_inode_mask_locked(inode);
	spin_unlock(&inode->i_lock);

	__fsnotify_update_child_dentry_flags(inode);
}

void fsnotify_destroy_mark_by_entry(struct fsnotify_mark_entry *entry)
{
	struct fsnotify_group *group;
	struct inode *inode;

	spin_lock(&entry->lock);

	group = entry->group;
	inode = entry->inode;

	BUG_ON(group && !inode);
	BUG_ON(!group && inode);

	/* if !group something else already marked this to die */
	if (!group) {
		spin_unlock(&entry->lock);
		return;
	}

	/* 1 from caller and 1 for being on i_list/g_list */
	BUG_ON(atomic_read(&entry->refcnt) < 2);

	spin_lock(&group->mark_lock);
	spin_lock(&inode->i_lock);

	hlist_del_init(&entry->i_list);
	entry->inode = NULL;

	list_del_init(&entry->g_list);
	entry->group = NULL;

	fsnotify_put_mark(entry); /* for i_list and g_list */

	/*
	 * this mark is now off the inode->i_fsnotify_mark_entries list and we
	 * hold the inode->i_lock, so this is the perfect time to update the
	 * inode->i_fsnotify_mask
	 */
	fsnotify_recalc_inode_mask_locked(inode);

	spin_unlock(&inode->i_lock);
	spin_unlock(&group->mark_lock);
	spin_unlock(&entry->lock);

	/*
	 * Some groups like to know that marks are being freed.  This is a
	 * callback to the group function to let it know that this entry
	 * is being freed.
	 */
	if (group->ops->freeing_mark)
		group->ops->freeing_mark(entry, group);

	/*
	 * __fsnotify_update_child_dentry_flags(inode);
	 *
	 * I really want to call that, but we can't, we have no idea if the inode
	 * still exists the second we drop the entry->lock.
	 *
	 * The next time an event arrive to this inode from one of it's children
	 * __fsnotify_parent will see that the inode doesn't care about it's
	 * children and will update all of these flags then.  So really this
	 * is just a lazy update (and could be a perf win...)
	 */


	iput(inode);

	/*
	 * it's possible that this group tried to destroy itself, but this
	 * this mark was simultaneously being freed by inode.  If that's the
	 * case, we finish freeing the group here.
	 */
	if (unlikely(atomic_dec_and_test(&group->num_marks)))
		fsnotify_final_destroy_group(group);
}

void fsnotify_clear_marks_by_group(struct fsnotify_group *group)
{
	struct fsnotify_mark_entry *lentry, *entry;
	LIST_HEAD(free_list);

	spin_lock(&group->mark_lock);
	list_for_each_entry_safe(entry, lentry, &group->mark_entries, g_list) {
		list_add(&entry->free_g_list, &free_list);
		list_del_init(&entry->g_list);
		fsnotify_get_mark(entry);
	}
	spin_unlock(&group->mark_lock);

	list_for_each_entry_safe(entry, lentry, &free_list, free_g_list) {
		fsnotify_destroy_mark_by_entry(entry);
		fsnotify_put_mark(entry);
	}
}

void fsnotify_clear_marks_by_inode(struct inode *inode)
{
	struct fsnotify_mark_entry *entry, *lentry;
	struct hlist_node *pos, *n;
	LIST_HEAD(free_list);

	spin_lock(&inode->i_lock);
	hlist_for_each_entry_safe(entry, pos, n, &inode->i_fsnotify_mark_entries, i_list) {
		list_add(&entry->free_i_list, &free_list);
		hlist_del_init(&entry->i_list);
		fsnotify_get_mark(entry);
	}
	spin_unlock(&inode->i_lock);

	list_for_each_entry_safe(entry, lentry, &free_list, free_i_list) {
		fsnotify_destroy_mark_by_entry(entry);
		fsnotify_put_mark(entry);
	}
}

struct fsnotify_mark_entry *fsnotify_find_mark_entry(struct fsnotify_group *group,
						     struct inode *inode)
{
	struct fsnotify_mark_entry *entry;
	struct hlist_node *pos;

	assert_spin_locked(&inode->i_lock);

	hlist_for_each_entry(entry, pos, &inode->i_fsnotify_mark_entries, i_list) {
		if (entry->group == group) {
			fsnotify_get_mark(entry);
			return entry;
		}
	}
	return NULL;
}

void fsnotify_init_mark(struct fsnotify_mark_entry *entry,
			void (*free_mark)(struct fsnotify_mark_entry *entry))

{
	spin_lock_init(&entry->lock);
	atomic_set(&entry->refcnt, 1);
	INIT_HLIST_NODE(&entry->i_list);
	entry->group = NULL;
	entry->mask = 0;
	entry->inode = NULL;
	entry->free_mark = free_mark;
}

int fsnotify_add_mark(struct fsnotify_mark_entry *entry,
		      struct fsnotify_group *group, struct inode *inode)
{
	struct fsnotify_mark_entry *lentry;
	int ret = 0;

	inode = igrab(inode);
	if (unlikely(!inode))
		return -EINVAL;

	/*
	 * LOCKING ORDER!!!!
	 * entry->lock
	 * group->mark_lock
	 * inode->i_lock
	 */
	spin_lock(&entry->lock);
	spin_lock(&group->mark_lock);
	spin_lock(&inode->i_lock);

	lentry = fsnotify_find_mark_entry(group, inode);
	if (!lentry) {
		entry->group = group;
		entry->inode = inode;

		hlist_add_head(&entry->i_list, &inode->i_fsnotify_mark_entries);
		list_add(&entry->g_list, &group->mark_entries);

		fsnotify_get_mark(entry); /* for i_list and g_list */

		atomic_inc(&group->num_marks);

		fsnotify_recalc_inode_mask_locked(inode);
	}

	spin_unlock(&inode->i_lock);
	spin_unlock(&group->mark_lock);
	spin_unlock(&entry->lock);

	if (lentry) {
		ret = -EEXIST;
		iput(inode);
		fsnotify_put_mark(lentry);
	} else {
		__fsnotify_update_child_dentry_flags(inode);
	}

	return ret;
}

void fsnotify_unmount_inodes(struct list_head *list)
{
	struct inode *inode, *next_i, *need_iput = NULL;

	list_for_each_entry_safe(inode, next_i, list, i_sb_list) {
		struct inode *need_iput_tmp;

		/*
		 * We cannot __iget() an inode in state I_CLEAR, I_FREEING,
		 * I_WILL_FREE, or I_NEW which is fine because by that point
		 * the inode cannot have any associated watches.
		 */
		if (inode->i_state & (I_CLEAR|I_FREEING|I_WILL_FREE|I_NEW))
			continue;

		/*
		 * If i_count is zero, the inode cannot have any watches and
		 * doing an __iget/iput with MS_ACTIVE clear would actually
		 * evict all inodes with zero i_count from icache which is
		 * unnecessarily violent and may in fact be illegal to do.
		 */
		if (!atomic_read(&inode->i_count))
			continue;

		need_iput_tmp = need_iput;
		need_iput = NULL;

		/* In case fsnotify_inode_delete() drops a reference. */
		if (inode != need_iput_tmp)
			__iget(inode);
		else
			need_iput_tmp = NULL;

		/* In case the dropping of a reference would nuke next_i. */
		if ((&next_i->i_sb_list != list) &&
		    atomic_read(&next_i->i_count) &&
		    !(next_i->i_state & (I_CLEAR | I_FREEING | I_WILL_FREE))) {
			__iget(next_i);
			need_iput = next_i;
		}

		/*
		 * We can safely drop inode_lock here because we hold
		 * references on both inode and next_i.  Also no new inodes
		 * will be added since the umount has begun.  Finally,
		 * iprune_mutex keeps shrink_icache_memory() away.
		 */
		spin_unlock(&inode_lock);

		if (need_iput_tmp)
			iput(need_iput_tmp);

		/* for each watch, send FS_UNMOUNT and then remove it */
		fsnotify(inode, FS_UNMOUNT, inode, FSNOTIFY_EVENT_INODE, NULL, 0);

		fsnotify_inode_delete(inode);

		iput(inode);

		spin_lock(&inode_lock);
	}
}
