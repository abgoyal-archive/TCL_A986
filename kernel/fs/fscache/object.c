

#define FSCACHE_DEBUG_LEVEL COOKIE
#include <linux/module.h>
#include <linux/seq_file.h>
#include "internal.h"

const char *fscache_object_states[FSCACHE_OBJECT__NSTATES] = {
	[FSCACHE_OBJECT_INIT]		= "OBJECT_INIT",
	[FSCACHE_OBJECT_LOOKING_UP]	= "OBJECT_LOOKING_UP",
	[FSCACHE_OBJECT_CREATING]	= "OBJECT_CREATING",
	[FSCACHE_OBJECT_AVAILABLE]	= "OBJECT_AVAILABLE",
	[FSCACHE_OBJECT_ACTIVE]		= "OBJECT_ACTIVE",
	[FSCACHE_OBJECT_UPDATING]	= "OBJECT_UPDATING",
	[FSCACHE_OBJECT_DYING]		= "OBJECT_DYING",
	[FSCACHE_OBJECT_LC_DYING]	= "OBJECT_LC_DYING",
	[FSCACHE_OBJECT_ABORT_INIT]	= "OBJECT_ABORT_INIT",
	[FSCACHE_OBJECT_RELEASING]	= "OBJECT_RELEASING",
	[FSCACHE_OBJECT_RECYCLING]	= "OBJECT_RECYCLING",
	[FSCACHE_OBJECT_WITHDRAWING]	= "OBJECT_WITHDRAWING",
	[FSCACHE_OBJECT_DEAD]		= "OBJECT_DEAD",
};
EXPORT_SYMBOL(fscache_object_states);

const char fscache_object_states_short[FSCACHE_OBJECT__NSTATES][5] = {
	[FSCACHE_OBJECT_INIT]		= "INIT",
	[FSCACHE_OBJECT_LOOKING_UP]	= "LOOK",
	[FSCACHE_OBJECT_CREATING]	= "CRTN",
	[FSCACHE_OBJECT_AVAILABLE]	= "AVBL",
	[FSCACHE_OBJECT_ACTIVE]		= "ACTV",
	[FSCACHE_OBJECT_UPDATING]	= "UPDT",
	[FSCACHE_OBJECT_DYING]		= "DYNG",
	[FSCACHE_OBJECT_LC_DYING]	= "LCDY",
	[FSCACHE_OBJECT_ABORT_INIT]	= "ABTI",
	[FSCACHE_OBJECT_RELEASING]	= "RELS",
	[FSCACHE_OBJECT_RECYCLING]	= "RCYC",
	[FSCACHE_OBJECT_WITHDRAWING]	= "WTHD",
	[FSCACHE_OBJECT_DEAD]		= "DEAD",
};

static void fscache_object_slow_work_put_ref(struct slow_work *);
static int  fscache_object_slow_work_get_ref(struct slow_work *);
static void fscache_object_slow_work_execute(struct slow_work *);
#ifdef CONFIG_SLOW_WORK_DEBUG
static void fscache_object_slow_work_desc(struct slow_work *, struct seq_file *);
#endif
static void fscache_initialise_object(struct fscache_object *);
static void fscache_lookup_object(struct fscache_object *);
static void fscache_object_available(struct fscache_object *);
static void fscache_release_object(struct fscache_object *);
static void fscache_withdraw_object(struct fscache_object *);
static void fscache_enqueue_dependents(struct fscache_object *);
static void fscache_dequeue_object(struct fscache_object *);

const struct slow_work_ops fscache_object_slow_work_ops = {
	.owner		= THIS_MODULE,
	.get_ref	= fscache_object_slow_work_get_ref,
	.put_ref	= fscache_object_slow_work_put_ref,
	.execute	= fscache_object_slow_work_execute,
#ifdef CONFIG_SLOW_WORK_DEBUG
	.desc		= fscache_object_slow_work_desc,
#endif
};
EXPORT_SYMBOL(fscache_object_slow_work_ops);

static inline void fscache_done_parent_op(struct fscache_object *object)
{
	struct fscache_object *parent = object->parent;

	_enter("OBJ%x {OBJ%x,%x}",
	       object->debug_id, parent->debug_id, parent->n_ops);

	spin_lock_nested(&parent->lock, 1);
	parent->n_ops--;
	parent->n_obj_ops--;
	if (parent->n_ops == 0)
		fscache_raise_event(parent, FSCACHE_OBJECT_EV_CLEARED);
	spin_unlock(&parent->lock);
}

static void fscache_object_state_machine(struct fscache_object *object)
{
	enum fscache_object_state new_state;
	struct fscache_cookie *cookie;

	ASSERT(object != NULL);

	_enter("{OBJ%x,%s,%lx}",
	       object->debug_id, fscache_object_states[object->state],
	       object->events);

	switch (object->state) {
		/* wait for the parent object to become ready */
	case FSCACHE_OBJECT_INIT:
		object->event_mask =
			ULONG_MAX & ~(1 << FSCACHE_OBJECT_EV_CLEARED);
		fscache_initialise_object(object);
		goto done;

		/* look up the object metadata on disk */
	case FSCACHE_OBJECT_LOOKING_UP:
		fscache_lookup_object(object);
		goto lookup_transit;

		/* create the object metadata on disk */
	case FSCACHE_OBJECT_CREATING:
		fscache_lookup_object(object);
		goto lookup_transit;

		/* handle an object becoming available; start pending
		 * operations and queue dependent operations for processing */
	case FSCACHE_OBJECT_AVAILABLE:
		fscache_object_available(object);
		goto active_transit;

		/* normal running state */
	case FSCACHE_OBJECT_ACTIVE:
		goto active_transit;

		/* update the object metadata on disk */
	case FSCACHE_OBJECT_UPDATING:
		clear_bit(FSCACHE_OBJECT_EV_UPDATE, &object->events);
		fscache_stat(&fscache_n_updates_run);
		fscache_stat(&fscache_n_cop_update_object);
		object->cache->ops->update_object(object);
		fscache_stat_d(&fscache_n_cop_update_object);
		goto active_transit;

		/* handle an object dying during lookup or creation */
	case FSCACHE_OBJECT_LC_DYING:
		object->event_mask &= ~(1 << FSCACHE_OBJECT_EV_UPDATE);
		fscache_stat(&fscache_n_cop_lookup_complete);
		object->cache->ops->lookup_complete(object);
		fscache_stat_d(&fscache_n_cop_lookup_complete);

		spin_lock(&object->lock);
		object->state = FSCACHE_OBJECT_DYING;
		cookie = object->cookie;
		if (cookie) {
			if (test_and_clear_bit(FSCACHE_COOKIE_LOOKING_UP,
					       &cookie->flags))
				wake_up_bit(&cookie->flags,
					    FSCACHE_COOKIE_LOOKING_UP);
			if (test_and_clear_bit(FSCACHE_COOKIE_CREATING,
					       &cookie->flags))
				wake_up_bit(&cookie->flags,
					    FSCACHE_COOKIE_CREATING);
		}
		spin_unlock(&object->lock);

		fscache_done_parent_op(object);

		/* wait for completion of all active operations on this object
		 * and the death of all child objects of this object */
	case FSCACHE_OBJECT_DYING:
	dying:
		clear_bit(FSCACHE_OBJECT_EV_CLEARED, &object->events);
		spin_lock(&object->lock);
		_debug("dying OBJ%x {%d,%d}",
		       object->debug_id, object->n_ops, object->n_children);
		if (object->n_ops == 0 && object->n_children == 0) {
			object->event_mask &=
				~(1 << FSCACHE_OBJECT_EV_CLEARED);
			object->event_mask |=
				(1 << FSCACHE_OBJECT_EV_WITHDRAW) |
				(1 << FSCACHE_OBJECT_EV_RETIRE) |
				(1 << FSCACHE_OBJECT_EV_RELEASE) |
				(1 << FSCACHE_OBJECT_EV_ERROR);
		} else {
			object->event_mask &=
				~((1 << FSCACHE_OBJECT_EV_WITHDRAW) |
				  (1 << FSCACHE_OBJECT_EV_RETIRE) |
				  (1 << FSCACHE_OBJECT_EV_RELEASE) |
				  (1 << FSCACHE_OBJECT_EV_ERROR));
			object->event_mask |=
				1 << FSCACHE_OBJECT_EV_CLEARED;
		}
		spin_unlock(&object->lock);
		fscache_enqueue_dependents(object);
		fscache_start_operations(object);
		goto terminal_transit;

		/* handle an abort during initialisation */
	case FSCACHE_OBJECT_ABORT_INIT:
		_debug("handle abort init %lx", object->events);
		object->event_mask &= ~(1 << FSCACHE_OBJECT_EV_UPDATE);

		spin_lock(&object->lock);
		fscache_dequeue_object(object);

		object->state = FSCACHE_OBJECT_DYING;
		if (test_and_clear_bit(FSCACHE_COOKIE_CREATING,
				       &object->cookie->flags))
			wake_up_bit(&object->cookie->flags,
				    FSCACHE_COOKIE_CREATING);
		spin_unlock(&object->lock);
		goto dying;

		/* handle the netfs releasing an object and possibly marking it
		 * obsolete too */
	case FSCACHE_OBJECT_RELEASING:
	case FSCACHE_OBJECT_RECYCLING:
		object->event_mask &=
			~((1 << FSCACHE_OBJECT_EV_WITHDRAW) |
			  (1 << FSCACHE_OBJECT_EV_RETIRE) |
			  (1 << FSCACHE_OBJECT_EV_RELEASE) |
			  (1 << FSCACHE_OBJECT_EV_ERROR));
		fscache_release_object(object);
		spin_lock(&object->lock);
		object->state = FSCACHE_OBJECT_DEAD;
		spin_unlock(&object->lock);
		fscache_stat(&fscache_n_object_dead);
		goto terminal_transit;

		/* handle the parent cache of this object being withdrawn from
		 * active service */
	case FSCACHE_OBJECT_WITHDRAWING:
		object->event_mask &=
			~((1 << FSCACHE_OBJECT_EV_WITHDRAW) |
			  (1 << FSCACHE_OBJECT_EV_RETIRE) |
			  (1 << FSCACHE_OBJECT_EV_RELEASE) |
			  (1 << FSCACHE_OBJECT_EV_ERROR));
		fscache_withdraw_object(object);
		spin_lock(&object->lock);
		object->state = FSCACHE_OBJECT_DEAD;
		spin_unlock(&object->lock);
		fscache_stat(&fscache_n_object_dead);
		goto terminal_transit;

		/* complain about the object being woken up once it is
		 * deceased */
	case FSCACHE_OBJECT_DEAD:
		printk(KERN_ERR "FS-Cache:"
		       " Unexpected event in dead state %lx\n",
		       object->events & object->event_mask);
		BUG();

	default:
		printk(KERN_ERR "FS-Cache: Unknown object state %u\n",
		       object->state);
		BUG();
	}

	/* determine the transition from a lookup state */
lookup_transit:
	switch (fls(object->events & object->event_mask) - 1) {
	case FSCACHE_OBJECT_EV_WITHDRAW:
	case FSCACHE_OBJECT_EV_RETIRE:
	case FSCACHE_OBJECT_EV_RELEASE:
	case FSCACHE_OBJECT_EV_ERROR:
		new_state = FSCACHE_OBJECT_LC_DYING;
		goto change_state;
	case FSCACHE_OBJECT_EV_REQUEUE:
		goto done;
	case -1:
		goto done; /* sleep until event */
	default:
		goto unsupported_event;
	}

	/* determine the transition from an active state */
active_transit:
	switch (fls(object->events & object->event_mask) - 1) {
	case FSCACHE_OBJECT_EV_WITHDRAW:
	case FSCACHE_OBJECT_EV_RETIRE:
	case FSCACHE_OBJECT_EV_RELEASE:
	case FSCACHE_OBJECT_EV_ERROR:
		new_state = FSCACHE_OBJECT_DYING;
		goto change_state;
	case FSCACHE_OBJECT_EV_UPDATE:
		new_state = FSCACHE_OBJECT_UPDATING;
		goto change_state;
	case -1:
		new_state = FSCACHE_OBJECT_ACTIVE;
		goto change_state; /* sleep until event */
	default:
		goto unsupported_event;
	}

	/* determine the transition from a terminal state */
terminal_transit:
	switch (fls(object->events & object->event_mask) - 1) {
	case FSCACHE_OBJECT_EV_WITHDRAW:
		new_state = FSCACHE_OBJECT_WITHDRAWING;
		goto change_state;
	case FSCACHE_OBJECT_EV_RETIRE:
		new_state = FSCACHE_OBJECT_RECYCLING;
		goto change_state;
	case FSCACHE_OBJECT_EV_RELEASE:
		new_state = FSCACHE_OBJECT_RELEASING;
		goto change_state;
	case FSCACHE_OBJECT_EV_ERROR:
		new_state = FSCACHE_OBJECT_WITHDRAWING;
		goto change_state;
	case FSCACHE_OBJECT_EV_CLEARED:
		new_state = FSCACHE_OBJECT_DYING;
		goto change_state;
	case -1:
		goto done; /* sleep until event */
	default:
		goto unsupported_event;
	}

change_state:
	spin_lock(&object->lock);
	object->state = new_state;
	spin_unlock(&object->lock);

done:
	_leave(" [->%s]", fscache_object_states[object->state]);
	return;

unsupported_event:
	printk(KERN_ERR "FS-Cache:"
	       " Unsupported event %lx [mask %lx] in state %s\n",
	       object->events, object->event_mask,
	       fscache_object_states[object->state]);
	BUG();
}

static void fscache_object_slow_work_execute(struct slow_work *work)
{
	struct fscache_object *object =
		container_of(work, struct fscache_object, work);
	unsigned long start;

	_enter("{OBJ%x}", object->debug_id);

	start = jiffies;
	fscache_object_state_machine(object);
	fscache_hist(fscache_objs_histogram, start);
	if (object->events & object->event_mask)
		fscache_enqueue_object(object);
	clear_bit(FSCACHE_OBJECT_EV_REQUEUE, &object->events);
}

#ifdef CONFIG_SLOW_WORK_DEBUG
static void fscache_object_slow_work_desc(struct slow_work *work,
					  struct seq_file *m)
{
	struct fscache_object *object =
		container_of(work, struct fscache_object, work);

	seq_printf(m, "FSC: OBJ%x: %s",
		   object->debug_id,
		   fscache_object_states_short[object->state]);
}
#endif

static void fscache_initialise_object(struct fscache_object *object)
{
	struct fscache_object *parent;

	_enter("");
	ASSERT(object->cookie != NULL);
	ASSERT(object->cookie->parent != NULL);
	ASSERT(list_empty(&object->work.link));

	if (object->events & ((1 << FSCACHE_OBJECT_EV_ERROR) |
			      (1 << FSCACHE_OBJECT_EV_RELEASE) |
			      (1 << FSCACHE_OBJECT_EV_RETIRE) |
			      (1 << FSCACHE_OBJECT_EV_WITHDRAW))) {
		_debug("abort init %lx", object->events);
		spin_lock(&object->lock);
		object->state = FSCACHE_OBJECT_ABORT_INIT;
		spin_unlock(&object->lock);
		return;
	}

	spin_lock(&object->cookie->lock);
	spin_lock_nested(&object->cookie->parent->lock, 1);

	parent = object->parent;
	if (!parent) {
		_debug("no parent");
		set_bit(FSCACHE_OBJECT_EV_WITHDRAW, &object->events);
	} else {
		spin_lock(&object->lock);
		spin_lock_nested(&parent->lock, 1);
		_debug("parent %s", fscache_object_states[parent->state]);

		if (parent->state >= FSCACHE_OBJECT_DYING) {
			_debug("bad parent");
			set_bit(FSCACHE_OBJECT_EV_WITHDRAW, &object->events);
		} else if (parent->state < FSCACHE_OBJECT_AVAILABLE) {
			_debug("wait");

			/* we may get woken up in this state by child objects
			 * binding on to us, so we need to make sure we don't
			 * add ourself to the list multiple times */
			if (list_empty(&object->dep_link)) {
				fscache_stat(&fscache_n_cop_grab_object);
				object->cache->ops->grab_object(object);
				fscache_stat_d(&fscache_n_cop_grab_object);
				list_add(&object->dep_link,
					 &parent->dependents);

				/* fscache_acquire_non_index_cookie() uses this
				 * to wake the chain up */
				if (parent->state == FSCACHE_OBJECT_INIT)
					fscache_enqueue_object(parent);
			}
		} else {
			_debug("go");
			parent->n_ops++;
			parent->n_obj_ops++;
			object->lookup_jif = jiffies;
			object->state = FSCACHE_OBJECT_LOOKING_UP;
			set_bit(FSCACHE_OBJECT_EV_REQUEUE, &object->events);
		}

		spin_unlock(&parent->lock);
		spin_unlock(&object->lock);
	}

	spin_unlock(&object->cookie->parent->lock);
	spin_unlock(&object->cookie->lock);
	_leave("");
}

static void fscache_lookup_object(struct fscache_object *object)
{
	struct fscache_cookie *cookie = object->cookie;
	struct fscache_object *parent;
	int ret;

	_enter("");

	parent = object->parent;
	ASSERT(parent != NULL);
	ASSERTCMP(parent->n_ops, >, 0);
	ASSERTCMP(parent->n_obj_ops, >, 0);

	/* make sure the parent is still available */
	ASSERTCMP(parent->state, >=, FSCACHE_OBJECT_AVAILABLE);

	if (parent->state >= FSCACHE_OBJECT_DYING ||
	    test_bit(FSCACHE_IOERROR, &object->cache->flags)) {
		_debug("unavailable");
		set_bit(FSCACHE_OBJECT_EV_WITHDRAW, &object->events);
		_leave("");
		return;
	}

	_debug("LOOKUP \"%s/%s\" in \"%s\"",
	       parent->cookie->def->name, cookie->def->name,
	       object->cache->tag->name);

	fscache_stat(&fscache_n_object_lookups);
	fscache_stat(&fscache_n_cop_lookup_object);
	ret = object->cache->ops->lookup_object(object);
	fscache_stat_d(&fscache_n_cop_lookup_object);

	if (test_bit(FSCACHE_OBJECT_EV_ERROR, &object->events))
		set_bit(FSCACHE_COOKIE_UNAVAILABLE, &cookie->flags);

	if (ret == -ETIMEDOUT) {
		/* probably stuck behind another object, so move this one to
		 * the back of the queue */
		fscache_stat(&fscache_n_object_lookups_timed_out);
		set_bit(FSCACHE_OBJECT_EV_REQUEUE, &object->events);
	}

	_leave("");
}

void fscache_object_lookup_negative(struct fscache_object *object)
{
	struct fscache_cookie *cookie = object->cookie;

	_enter("{OBJ%x,%s}",
	       object->debug_id, fscache_object_states[object->state]);

	spin_lock(&object->lock);
	if (object->state == FSCACHE_OBJECT_LOOKING_UP) {
		fscache_stat(&fscache_n_object_lookups_negative);

		/* transit here to allow write requests to begin stacking up
		 * and read requests to begin returning ENODATA */
		object->state = FSCACHE_OBJECT_CREATING;
		spin_unlock(&object->lock);

		set_bit(FSCACHE_COOKIE_PENDING_FILL, &cookie->flags);
		set_bit(FSCACHE_COOKIE_NO_DATA_YET, &cookie->flags);

		_debug("wake up lookup %p", &cookie->flags);
		smp_mb__before_clear_bit();
		clear_bit(FSCACHE_COOKIE_LOOKING_UP, &cookie->flags);
		smp_mb__after_clear_bit();
		wake_up_bit(&cookie->flags, FSCACHE_COOKIE_LOOKING_UP);
		set_bit(FSCACHE_OBJECT_EV_REQUEUE, &object->events);
	} else {
		ASSERTCMP(object->state, ==, FSCACHE_OBJECT_CREATING);
		spin_unlock(&object->lock);
	}

	_leave("");
}
EXPORT_SYMBOL(fscache_object_lookup_negative);

void fscache_obtained_object(struct fscache_object *object)
{
	struct fscache_cookie *cookie = object->cookie;

	_enter("{OBJ%x,%s}",
	       object->debug_id, fscache_object_states[object->state]);

	/* if we were still looking up, then we must have a positive lookup
	 * result, in which case there may be data available */
	spin_lock(&object->lock);
	if (object->state == FSCACHE_OBJECT_LOOKING_UP) {
		fscache_stat(&fscache_n_object_lookups_positive);

		clear_bit(FSCACHE_COOKIE_NO_DATA_YET, &cookie->flags);

		object->state = FSCACHE_OBJECT_AVAILABLE;
		spin_unlock(&object->lock);

		smp_mb__before_clear_bit();
		clear_bit(FSCACHE_COOKIE_LOOKING_UP, &cookie->flags);
		smp_mb__after_clear_bit();
		wake_up_bit(&cookie->flags, FSCACHE_COOKIE_LOOKING_UP);
		set_bit(FSCACHE_OBJECT_EV_REQUEUE, &object->events);
	} else {
		ASSERTCMP(object->state, ==, FSCACHE_OBJECT_CREATING);
		fscache_stat(&fscache_n_object_created);

		object->state = FSCACHE_OBJECT_AVAILABLE;
		spin_unlock(&object->lock);
		set_bit(FSCACHE_OBJECT_EV_REQUEUE, &object->events);
		smp_wmb();
	}

	if (test_and_clear_bit(FSCACHE_COOKIE_CREATING, &cookie->flags))
		wake_up_bit(&cookie->flags, FSCACHE_COOKIE_CREATING);

	_leave("");
}
EXPORT_SYMBOL(fscache_obtained_object);

static void fscache_object_available(struct fscache_object *object)
{
	_enter("{OBJ%x}", object->debug_id);

	spin_lock(&object->lock);

	if (object->cookie &&
	    test_and_clear_bit(FSCACHE_COOKIE_CREATING, &object->cookie->flags))
		wake_up_bit(&object->cookie->flags, FSCACHE_COOKIE_CREATING);

	fscache_done_parent_op(object);
	if (object->n_in_progress == 0) {
		if (object->n_ops > 0) {
			ASSERTCMP(object->n_ops, >=, object->n_obj_ops);
			ASSERTIF(object->n_ops > object->n_obj_ops,
				 !list_empty(&object->pending_ops));
			fscache_start_operations(object);
		} else {
			ASSERT(list_empty(&object->pending_ops));
		}
	}
	spin_unlock(&object->lock);

	fscache_stat(&fscache_n_cop_lookup_complete);
	object->cache->ops->lookup_complete(object);
	fscache_stat_d(&fscache_n_cop_lookup_complete);
	fscache_enqueue_dependents(object);

	fscache_hist(fscache_obj_instantiate_histogram, object->lookup_jif);
	fscache_stat(&fscache_n_object_avail);

	_leave("");
}

static void fscache_drop_object(struct fscache_object *object)
{
	struct fscache_object *parent = object->parent;
	struct fscache_cache *cache = object->cache;

	_enter("{OBJ%x,%d}", object->debug_id, object->n_children);

	ASSERTCMP(object->cookie, ==, NULL);
	ASSERT(hlist_unhashed(&object->cookie_link));

	spin_lock(&cache->object_list_lock);
	list_del_init(&object->cache_link);
	spin_unlock(&cache->object_list_lock);

	fscache_stat(&fscache_n_cop_drop_object);
	cache->ops->drop_object(object);
	fscache_stat_d(&fscache_n_cop_drop_object);

	if (parent) {
		_debug("release parent OBJ%x {%d}",
		       parent->debug_id, parent->n_children);

		spin_lock(&parent->lock);
		parent->n_children--;
		if (parent->n_children == 0)
			fscache_raise_event(parent, FSCACHE_OBJECT_EV_CLEARED);
		spin_unlock(&parent->lock);
		object->parent = NULL;
	}

	/* this just shifts the object release to the slow work processor */
	fscache_stat(&fscache_n_cop_put_object);
	object->cache->ops->put_object(object);
	fscache_stat_d(&fscache_n_cop_put_object);

	_leave("");
}

static void fscache_release_object(struct fscache_object *object)
{
	_enter("");

	fscache_drop_object(object);
}

static void fscache_withdraw_object(struct fscache_object *object)
{
	struct fscache_cookie *cookie;
	bool detached;

	_enter("");

	spin_lock(&object->lock);
	cookie = object->cookie;
	if (cookie) {
		/* need to get the cookie lock before the object lock, starting
		 * from the object pointer */
		atomic_inc(&cookie->usage);
		spin_unlock(&object->lock);

		detached = false;
		spin_lock(&cookie->lock);
		spin_lock(&object->lock);

		if (object->cookie == cookie) {
			hlist_del_init(&object->cookie_link);
			object->cookie = NULL;
			detached = true;
		}
		spin_unlock(&cookie->lock);
		fscache_cookie_put(cookie);
		if (detached)
			fscache_cookie_put(cookie);
	}

	spin_unlock(&object->lock);

	fscache_drop_object(object);
}

void fscache_withdrawing_object(struct fscache_cache *cache,
				struct fscache_object *object)
{
	bool enqueue = false;

	_enter(",OBJ%x", object->debug_id);

	spin_lock(&object->lock);
	if (object->state < FSCACHE_OBJECT_WITHDRAWING) {
		object->state = FSCACHE_OBJECT_WITHDRAWING;
		enqueue = true;
	}
	spin_unlock(&object->lock);

	if (enqueue)
		fscache_enqueue_object(object);

	_leave("");
}

static int fscache_object_slow_work_get_ref(struct slow_work *work)
{
	struct fscache_object *object =
		container_of(work, struct fscache_object, work);
	int ret;

	fscache_stat(&fscache_n_cop_grab_object);
	ret = object->cache->ops->grab_object(object) ? 0 : -EAGAIN;
	fscache_stat_d(&fscache_n_cop_grab_object);
	return ret;
}

static void fscache_object_slow_work_put_ref(struct slow_work *work)
{
	struct fscache_object *object =
		container_of(work, struct fscache_object, work);

	fscache_stat(&fscache_n_cop_put_object);
	object->cache->ops->put_object(object);
	fscache_stat_d(&fscache_n_cop_put_object);
}

void fscache_enqueue_object(struct fscache_object *object)
{
	_enter("{OBJ%x}", object->debug_id);

	slow_work_enqueue(&object->work);
}

static void fscache_enqueue_dependents(struct fscache_object *object)
{
	struct fscache_object *dep;

	_enter("{OBJ%x}", object->debug_id);

	if (list_empty(&object->dependents))
		return;

	spin_lock(&object->lock);

	while (!list_empty(&object->dependents)) {
		dep = list_entry(object->dependents.next,
				 struct fscache_object, dep_link);
		list_del_init(&dep->dep_link);


		/* sort onto appropriate lists */
		fscache_enqueue_object(dep);
		fscache_stat(&fscache_n_cop_put_object);
		dep->cache->ops->put_object(dep);
		fscache_stat_d(&fscache_n_cop_put_object);

		if (!list_empty(&object->dependents))
			cond_resched_lock(&object->lock);
	}

	spin_unlock(&object->lock);
}

void fscache_dequeue_object(struct fscache_object *object)
{
	_enter("{OBJ%x}", object->debug_id);

	if (!list_empty(&object->dep_link)) {
		spin_lock(&object->parent->lock);
		list_del_init(&object->dep_link);
		spin_unlock(&object->parent->lock);
	}

	_leave("");
}

enum fscache_checkaux fscache_check_aux(struct fscache_object *object,
					const void *data, uint16_t datalen)
{
	enum fscache_checkaux result;

	if (!object->cookie->def->check_aux) {
		fscache_stat(&fscache_n_checkaux_none);
		return FSCACHE_CHECKAUX_OKAY;
	}

	result = object->cookie->def->check_aux(object->cookie->netfs_data,
						data, datalen);
	switch (result) {
		/* entry okay as is */
	case FSCACHE_CHECKAUX_OKAY:
		fscache_stat(&fscache_n_checkaux_okay);
		break;

		/* entry requires update */
	case FSCACHE_CHECKAUX_NEEDS_UPDATE:
		fscache_stat(&fscache_n_checkaux_update);
		break;

		/* entry requires deletion */
	case FSCACHE_CHECKAUX_OBSOLETE:
		fscache_stat(&fscache_n_checkaux_obsolete);
		break;

	default:
		BUG();
	}

	return result;
}
EXPORT_SYMBOL(fscache_check_aux);
