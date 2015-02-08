


#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/mount.h>
#include <linux/mutex.h>
#include <linux/namei.h>
#include <linux/path.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

#include <asm/atomic.h>

#include <linux/fsnotify_backend.h>
#include "fsnotify.h"

static struct kmem_cache *fsnotify_event_cachep;
static struct kmem_cache *fsnotify_event_holder_cachep;
static struct fsnotify_event q_overflow_event;
static atomic_t fsnotify_sync_cookie = ATOMIC_INIT(0);

u32 fsnotify_get_cookie(void)
{
	return atomic_inc_return(&fsnotify_sync_cookie);
}
EXPORT_SYMBOL_GPL(fsnotify_get_cookie);

/* return true if the notify queue is empty, false otherwise */
bool fsnotify_notify_queue_is_empty(struct fsnotify_group *group)
{
	BUG_ON(!mutex_is_locked(&group->notification_mutex));
	return list_empty(&group->notification_list) ? true : false;
}

void fsnotify_get_event(struct fsnotify_event *event)
{
	atomic_inc(&event->refcnt);
}

void fsnotify_put_event(struct fsnotify_event *event)
{
	if (!event)
		return;

	if (atomic_dec_and_test(&event->refcnt)) {
		if (event->data_type == FSNOTIFY_EVENT_PATH)
			path_put(&event->path);

		BUG_ON(!list_empty(&event->private_data_list));

		kfree(event->file_name);
		kmem_cache_free(fsnotify_event_cachep, event);
	}
}

struct fsnotify_event_holder *fsnotify_alloc_event_holder(void)
{
	return kmem_cache_alloc(fsnotify_event_holder_cachep, GFP_KERNEL);
}

void fsnotify_destroy_event_holder(struct fsnotify_event_holder *holder)
{
	kmem_cache_free(fsnotify_event_holder_cachep, holder);
}

struct fsnotify_event_private_data *fsnotify_remove_priv_from_event(struct fsnotify_group *group, struct fsnotify_event *event)
{
	struct fsnotify_event_private_data *lpriv;
	struct fsnotify_event_private_data *priv = NULL;

	assert_spin_locked(&event->lock);

	list_for_each_entry(lpriv, &event->private_data_list, event_list) {
		if (lpriv->group == group) {
			priv = lpriv;
			list_del(&priv->event_list);
			break;
		}
	}
	return priv;
}

static bool event_compare(struct fsnotify_event *old, struct fsnotify_event *new)
{
	if ((old->mask == new->mask) &&
	    (old->to_tell == new->to_tell) &&
	    (old->data_type == new->data_type) &&
	    (old->name_len == new->name_len)) {
		switch (old->data_type) {
		case (FSNOTIFY_EVENT_INODE):
			/* remember, after old was put on the wait_q we aren't
			 * allowed to look at the inode any more, only thing
			 * left to check was if the file_name is the same */
			if (!old->name_len ||
			    !strcmp(old->file_name, new->file_name))
				return true;
			break;
		case (FSNOTIFY_EVENT_PATH):
			if ((old->path.mnt == new->path.mnt) &&
			    (old->path.dentry == new->path.dentry))
				return true;
			break;
		case (FSNOTIFY_EVENT_NONE):
			if (old->mask & FS_Q_OVERFLOW)
				return true;
			else if (old->mask & FS_IN_IGNORED)
				return false;
			return false;
		};
	}
	return false;
}

int fsnotify_add_notify_event(struct fsnotify_group *group, struct fsnotify_event *event,
			      struct fsnotify_event_private_data *priv)
{
	struct fsnotify_event_holder *holder = NULL;
	struct list_head *list = &group->notification_list;
	struct fsnotify_event_holder *last_holder;
	struct fsnotify_event *last_event;
	int ret = 0;

	/*
	 * There is one fsnotify_event_holder embedded inside each fsnotify_event.
	 * Check if we expect to be able to use that holder.  If not alloc a new
	 * holder.
	 * For the overflow event it's possible that something will use the in
	 * event holder before we get the lock so we may need to jump back and
	 * alloc a new holder, this can't happen for most events...
	 */
	if (!list_empty(&event->holder.event_list)) {
alloc_holder:
		holder = fsnotify_alloc_event_holder();
		if (!holder)
			return -ENOMEM;
	}

	mutex_lock(&group->notification_mutex);

	if (group->q_len >= group->max_events) {
		event = &q_overflow_event;
		ret = -EOVERFLOW;
		/* sorry, no private data on the overflow event */
		priv = NULL;
	}

	spin_lock(&event->lock);

	if (list_empty(&event->holder.event_list)) {
		if (unlikely(holder))
			fsnotify_destroy_event_holder(holder);
		holder = &event->holder;
	} else if (unlikely(!holder)) {
		/* between the time we checked above and got the lock the in
		 * event holder was used, go back and get a new one */
		spin_unlock(&event->lock);
		mutex_unlock(&group->notification_mutex);
		goto alloc_holder;
	}

	if (!list_empty(list)) {
		last_holder = list_entry(list->prev, struct fsnotify_event_holder, event_list);
		last_event = last_holder->event;
		if (event_compare(last_event, event)) {
			spin_unlock(&event->lock);
			mutex_unlock(&group->notification_mutex);
			if (holder != &event->holder)
				fsnotify_destroy_event_holder(holder);
			return -EEXIST;
		}
	}

	group->q_len++;
	holder->event = event;

	fsnotify_get_event(event);
	list_add_tail(&holder->event_list, list);
	if (priv)
		list_add_tail(&priv->event_list, &event->private_data_list);
	spin_unlock(&event->lock);
	mutex_unlock(&group->notification_mutex);

	wake_up(&group->notification_waitq);
	return ret;
}

struct fsnotify_event *fsnotify_remove_notify_event(struct fsnotify_group *group)
{
	struct fsnotify_event *event;
	struct fsnotify_event_holder *holder;

	BUG_ON(!mutex_is_locked(&group->notification_mutex));

	holder = list_first_entry(&group->notification_list, struct fsnotify_event_holder, event_list);

	event = holder->event;

	spin_lock(&event->lock);
	holder->event = NULL;
	list_del_init(&holder->event_list);
	spin_unlock(&event->lock);

	/* event == holder means we are referenced through the in event holder */
	if (holder != &event->holder)
		fsnotify_destroy_event_holder(holder);

	group->q_len--;

	return event;
}

struct fsnotify_event *fsnotify_peek_notify_event(struct fsnotify_group *group)
{
	struct fsnotify_event *event;
	struct fsnotify_event_holder *holder;

	BUG_ON(!mutex_is_locked(&group->notification_mutex));

	holder = list_first_entry(&group->notification_list, struct fsnotify_event_holder, event_list);
	event = holder->event;

	return event;
}

void fsnotify_flush_notify(struct fsnotify_group *group)
{
	struct fsnotify_event *event;
	struct fsnotify_event_private_data *priv;

	mutex_lock(&group->notification_mutex);
	while (!fsnotify_notify_queue_is_empty(group)) {
		event = fsnotify_remove_notify_event(group);
		/* if they don't implement free_event_priv they better not have attached any */
		if (group->ops->free_event_priv) {
			spin_lock(&event->lock);
			priv = fsnotify_remove_priv_from_event(group, event);
			spin_unlock(&event->lock);
			if (priv)
				group->ops->free_event_priv(priv);
		}
		fsnotify_put_event(event); /* matches fsnotify_add_notify_event */
	}
	mutex_unlock(&group->notification_mutex);
}

static void initialize_event(struct fsnotify_event *event)
{
	event->holder.event = NULL;
	INIT_LIST_HEAD(&event->holder.event_list);
	atomic_set(&event->refcnt, 1);

	spin_lock_init(&event->lock);

	event->path.dentry = NULL;
	event->path.mnt = NULL;
	event->inode = NULL;
	event->data_type = FSNOTIFY_EVENT_NONE;

	INIT_LIST_HEAD(&event->private_data_list);

	event->to_tell = NULL;

	event->file_name = NULL;
	event->name_len = 0;

	event->sync_cookie = 0;
}

struct fsnotify_event *fsnotify_create_event(struct inode *to_tell, __u32 mask, void *data,
					     int data_type, const char *name, u32 cookie,
					     gfp_t gfp)
{
	struct fsnotify_event *event;

	event = kmem_cache_alloc(fsnotify_event_cachep, gfp);
	if (!event)
		return NULL;

	initialize_event(event);

	if (name) {
		event->file_name = kstrdup(name, gfp);
		if (!event->file_name) {
			kmem_cache_free(fsnotify_event_cachep, event);
			return NULL;
		}
		event->name_len = strlen(event->file_name);
	}

	event->sync_cookie = cookie;
	event->to_tell = to_tell;

	switch (data_type) {
	case FSNOTIFY_EVENT_FILE: {
		struct file *file = data;
		struct path *path = &file->f_path;
		event->path.dentry = path->dentry;
		event->path.mnt = path->mnt;
		path_get(&event->path);
		event->data_type = FSNOTIFY_EVENT_PATH;
		break;
	}
	case FSNOTIFY_EVENT_PATH: {
		struct path *path = data;
		event->path.dentry = path->dentry;
		event->path.mnt = path->mnt;
		path_get(&event->path);
		event->data_type = FSNOTIFY_EVENT_PATH;
		break;
	}
	case FSNOTIFY_EVENT_INODE:
		event->inode = data;
		event->data_type = FSNOTIFY_EVENT_INODE;
		break;
	case FSNOTIFY_EVENT_NONE:
		event->inode = NULL;
		event->path.dentry = NULL;
		event->path.mnt = NULL;
		break;
	default:
		BUG();
	}

	event->mask = mask;

	return event;
}

__init int fsnotify_notification_init(void)
{
	fsnotify_event_cachep = KMEM_CACHE(fsnotify_event, SLAB_PANIC);
	fsnotify_event_holder_cachep = KMEM_CACHE(fsnotify_event_holder, SLAB_PANIC);

	initialize_event(&q_overflow_event);
	q_overflow_event.mask = FS_Q_OVERFLOW;

	return 0;
}
subsys_initcall(fsnotify_notification_init);

