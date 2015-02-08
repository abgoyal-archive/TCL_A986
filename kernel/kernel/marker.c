
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/jhash.h>
#include <linux/hash.h>
#include <linux/list.h>
#include <linux/rcupdate.h>
#include <linux/marker.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/immediate.h>
#include <linux/ltt-channels.h>

extern struct marker __start___markers[];
extern struct marker __stop___markers[];

/* Set to 1 to enable marker debug output */
static const int marker_debug;

static DEFINE_MUTEX(markers_mutex);

void lock_markers(void)
{
	mutex_lock(&markers_mutex);
}
EXPORT_SYMBOL_GPL(lock_markers);

void unlock_markers(void)
{
	mutex_unlock(&markers_mutex);
}
EXPORT_SYMBOL_GPL(unlock_markers);

#define MARKER_HASH_BITS 6
#define MARKER_TABLE_SIZE (1 << MARKER_HASH_BITS)
static struct hlist_head marker_table[MARKER_TABLE_SIZE];
static struct hlist_head id_table[MARKER_TABLE_SIZE];

struct marker_probe_array {
	struct rcu_head rcu;
	struct marker_probe_closure c[0];
};

struct marker_entry {
	struct hlist_node hlist;
	struct hlist_node id_list;
	char *format;
	char *name;
			/* Probe wrapper */
	void (*call)(const struct marker *mdata, void *call_private, ...);
	struct marker_probe_closure single;
	struct marker_probe_array *multi;
	int refcount;	/* Number of times armed. 0 if disarmed. */
	u16 channel_id;
	u16 event_id;
	unsigned char ptype:1;
	unsigned char format_allocated:1;
	char channel[0];	/* Contains channel'\0'name'\0'format'\0' */
};

notrace void __mark_empty_function(const struct marker *mdata,
	void *probe_private, void *call_private, const char *fmt, va_list *args)
{
}
EXPORT_SYMBOL_GPL(__mark_empty_function);

notrace void marker_probe_cb(const struct marker *mdata,
		void *call_private, ...)
{
	va_list args;
	char ptype;

	/*
	 * rcu_read_lock_sched does two things : disabling preemption to make
	 * sure the teardown of the callbacks can be done correctly when they
	 * are in modules and they insure RCU read coherency.
	 */
	rcu_read_lock_sched_notrace();
	ptype = mdata->ptype;
	if (likely(!ptype)) {
		marker_probe_func *func;
		/* Must read the ptype before ptr. They are not data dependant,
		 * so we put an explicit smp_rmb() here. */
		smp_rmb();
		func = mdata->single.func;
		/* Must read the ptr before private data. They are not data
		 * dependant, so we put an explicit smp_rmb() here. */
		smp_rmb();
		va_start(args, call_private);
		func(mdata, mdata->single.probe_private, call_private,
			mdata->format, &args);
		va_end(args);
	} else {
		struct marker_probe_array *multi;
		int i;
		/*
		 * Read mdata->ptype before mdata->multi.
		 */
		smp_rmb();
		multi = mdata->multi;
		/*
		 * multi points to an array, therefore accessing the array
		 * depends on reading multi. However, even in this case,
		 * we must insure that the pointer is read _before_ the array
		 * data. Same as rcu_dereference, but we need a full smp_rmb()
		 * in the fast path, so put the explicit barrier here.
		 */
		smp_read_barrier_depends();
		for (i = 0; multi->c[i].func; i++) {
			va_start(args, call_private);
			multi->c[i].func(mdata, multi->c[i].probe_private,
				call_private, mdata->format, &args);
			va_end(args);
		}
	}
	rcu_read_unlock_sched_notrace();
}
EXPORT_SYMBOL_GPL(marker_probe_cb);

static notrace void marker_probe_cb_noarg(const struct marker *mdata,
		void *call_private, ...)
{
	va_list args;	/* not initialized */
	char ptype;

	rcu_read_lock_sched_notrace();
	ptype = mdata->ptype;
	if (likely(!ptype)) {
		marker_probe_func *func;
		/* Must read the ptype before ptr. They are not data dependant,
		 * so we put an explicit smp_rmb() here. */
		smp_rmb();
		func = mdata->single.func;
		/* Must read the ptr before private data. They are not data
		 * dependant, so we put an explicit smp_rmb() here. */
		smp_rmb();
		func(mdata, mdata->single.probe_private, call_private,
			mdata->format, &args);
	} else {
		struct marker_probe_array *multi;
		int i;
		/*
		 * Read mdata->ptype before mdata->multi.
		 */
		smp_rmb();
		multi = mdata->multi;
		/*
		 * multi points to an array, therefore accessing the array
		 * depends on reading multi. However, even in this case,
		 * we must insure that the pointer is read _before_ the array
		 * data. Same as rcu_dereference, but we need a full smp_rmb()
		 * in the fast path, so put the explicit barrier here.
		 */
		smp_read_barrier_depends();
		for (i = 0; multi->c[i].func; i++)
			multi->c[i].func(mdata, multi->c[i].probe_private,
				call_private, mdata->format, &args);
	}
	rcu_read_unlock_sched_notrace();
}

static void free_old_closure(struct rcu_head *head)
{
	struct marker_probe_array *multi = container_of(head, struct marker_probe_array, rcu);
	kfree(multi);
}

static void debug_print_probes(struct marker_entry *entry)
{
	int i;

	if (!marker_debug)
		return;

	if (!entry->ptype) {
		printk(KERN_DEBUG "Single probe : %p %p\n",
			entry->single.func,
			entry->single.probe_private);
	} else {
		for (i = 0; entry->multi->c[i].func; i++)
			printk(KERN_DEBUG "Multi probe %d : %p %p\n", i,
				entry->multi->c[i].func,
				entry->multi->c[i].probe_private);
	}
}

static struct marker_probe_array *
marker_entry_add_probe(struct marker_entry *entry,
		marker_probe_func *probe, void *probe_private)
{
	int nr_probes = 0;
	struct marker_probe_array *old, *new;

	WARN_ON(!probe);

	debug_print_probes(entry);
	old = entry->multi;
	if (!entry->ptype) {
		if (entry->single.func == probe &&
				entry->single.probe_private == probe_private)
			return ERR_PTR(-EBUSY);
		if (entry->single.func == __mark_empty_function) {
			/* 0 -> 1 probes */
			entry->single.func = probe;
			entry->single.probe_private = probe_private;
			entry->refcount = 1;
			entry->ptype = 0;
			debug_print_probes(entry);
			return NULL;
		} else {
			/* 1 -> 2 probes */
			nr_probes = 1;
			old = NULL;
		}
	} else {
		/* (N -> N+1), (N != 0, 1) probes */
		for (nr_probes = 0; old->c[nr_probes].func; nr_probes++)
			if (old->c[nr_probes].func == probe
					&& old->c[nr_probes].probe_private
						== probe_private)
				return ERR_PTR(-EBUSY);
	}
	/* + 2 : one for new probe, one for NULL func */
	new = kzalloc(sizeof(struct marker_probe_array)
		      + ((nr_probes + 2) * sizeof(struct marker_probe_closure)),
			GFP_KERNEL);
	if (new == NULL)
		return ERR_PTR(-ENOMEM);
	INIT_RCU_HEAD(&new->rcu);
	if (!old)
		new->c[0] = entry->single;
	else
		memcpy(&new->c[0], &old->c[0],
			nr_probes * sizeof(struct marker_probe_closure));
	new->c[nr_probes].func = probe;
	new->c[nr_probes].probe_private = probe_private;
	entry->refcount = nr_probes + 1;
	entry->multi = new;
	entry->ptype = 1;
	debug_print_probes(entry);
	return old;
}

static struct marker_probe_array *
marker_entry_remove_probe(struct marker_entry *entry,
		marker_probe_func *probe, void *probe_private)
{
	int nr_probes = 0, nr_del = 0, i;
	struct marker_probe_array *old, *new;

	old = entry->multi;

	debug_print_probes(entry);
	if (!entry->ptype) {
		/* 0 -> N is an error */
		WARN_ON(entry->single.func == __mark_empty_function);
		/* 1 -> 0 probes */
		WARN_ON(probe && entry->single.func != probe);
		WARN_ON(entry->single.probe_private != probe_private);
		entry->single.func = __mark_empty_function;
		entry->refcount = 0;
		entry->ptype = 0;
		debug_print_probes(entry);
		return NULL;
	} else {
		/* (N -> M), (N > 1, M >= 0) probes */
		for (nr_probes = 0; old->c[nr_probes].func; nr_probes++) {
			if ((!probe || old->c[nr_probes].func == probe)
					&& old->c[nr_probes].probe_private
						== probe_private)
				nr_del++;
		}
	}

	if (nr_probes - nr_del == 0) {
		/* N -> 0, (N > 1) */
		entry->single.func = __mark_empty_function;
		entry->refcount = 0;
		entry->ptype = 0;
	} else if (nr_probes - nr_del == 1) {
		/* N -> 1, (N > 1) */
		for (i = 0; old->c[i].func; i++)
			if ((probe && old->c[i].func != probe) ||
			    old->c[i].probe_private != probe_private)
				entry->single = old->c[i];
		entry->refcount = 1;
		entry->ptype = 0;
	} else {
		int j = 0;
		/* N -> M, (N > 1, M > 1) */
		/* + 1 for NULL */
		new = kzalloc(sizeof(struct marker_probe_array)
			      + ((nr_probes - nr_del + 1)
			         * sizeof(struct marker_probe_closure)),
			      GFP_KERNEL);
		if (new == NULL)
			return ERR_PTR(-ENOMEM);
		INIT_RCU_HEAD(&new->rcu);
		for (i = 0; old->c[i].func; i++)
			if ((probe && old->c[i].func != probe) ||
			    old->c[i].probe_private != probe_private)
				new->c[j++] = old->c[i];
		entry->refcount = nr_probes - nr_del;
		entry->ptype = 1;
		entry->multi = new;
	}
	debug_print_probes(entry);
	return old;
}

static struct marker_entry *get_marker(const char *channel, const char *name)
{
	struct hlist_head *head;
	struct hlist_node *node;
	struct marker_entry *e;
	size_t channel_len = strlen(channel) + 1;
	size_t name_len = strlen(name) + 1;
	u32 hash;

	hash = jhash(channel, channel_len-1, 0) ^ jhash(name, name_len-1, 0);
	head = &marker_table[hash & ((1 << MARKER_HASH_BITS)-1)];
	hlist_for_each_entry(e, node, head, hlist) {
		if (!strcmp(channel, e->channel) && !strcmp(name, e->name))
			return e;
	}
	return NULL;
}

static struct marker_entry *add_marker(const char *channel, const char *name,
		const char *format)
{
	struct hlist_head *head;
	struct hlist_node *node;
	struct marker_entry *e;
	size_t channel_len = strlen(channel) + 1;
	size_t name_len = strlen(name) + 1;
	size_t format_len = 0;
	u32 hash;

	hash = jhash(channel, channel_len-1, 0) ^ jhash(name, name_len-1, 0);
	if (format)
		format_len = strlen(format) + 1;
	head = &marker_table[hash & ((1 << MARKER_HASH_BITS)-1)];
	hlist_for_each_entry(e, node, head, hlist) {
		if (!strcmp(channel, e->channel) && !strcmp(name, e->name)) {
			printk(KERN_NOTICE
				"Marker %s.%s busy\n", channel, name);
			return ERR_PTR(-EBUSY);	/* Already there */
		}
	}
	/*
	 * Using kmalloc here to allocate a variable length element. Could
	 * cause some memory fragmentation if overused.
	 */
	e = kmalloc(sizeof(struct marker_entry)
		    + channel_len + name_len + format_len,
		    GFP_KERNEL);
	if (!e)
		return ERR_PTR(-ENOMEM);
	memcpy(e->channel, channel, channel_len);
	e->name = &e->channel[channel_len];
	memcpy(e->name, name, name_len);
	if (format) {
		e->format = &e->name[name_len];
		memcpy(e->format, format, format_len);
		if (strcmp(e->format, MARK_NOARGS) == 0)
			e->call = marker_probe_cb_noarg;
		else
			e->call = marker_probe_cb;
		trace_mark(metadata, core_marker_format,
			   "channel %s name %s format %s",
			   e->channel, e->name, e->format);
	} else {
		e->format = NULL;
		e->call = marker_probe_cb;
	}
	e->single.func = __mark_empty_function;
	e->single.probe_private = NULL;
	e->multi = NULL;
	e->ptype = 0;
	e->format_allocated = 0;
	e->refcount = 0;
	hlist_add_head(&e->hlist, head);
	return e;
}

static int remove_marker(const char *channel, const char *name, int registered,
			 int compacting)
{
	struct hlist_head *head;
	struct hlist_node *node;
	struct marker_entry *e;
	int found = 0;
	size_t channel_len = strlen(channel) + 1;
	size_t name_len = strlen(name) + 1;
	u32 hash;
	int ret;

	hash = jhash(channel, channel_len-1, 0) ^ jhash(name, name_len-1, 0);
	head = &marker_table[hash & ((1 << MARKER_HASH_BITS)-1)];
	hlist_for_each_entry(e, node, head, hlist) {
		if (!strcmp(channel, e->channel) && !strcmp(name, e->name)) {
			found = 1;
			break;
		}
	}
	if (!found)
		return -ENOENT;
	if (e->single.func != __mark_empty_function)
		return -EBUSY;

	if (registered && ltt_channels_trace_ref())
		return 0;

	hlist_del(&e->hlist);
	hlist_del(&e->id_list);
	if (registered) {
		ret = ltt_channels_unregister(e->channel, compacting);
		WARN_ON(ret);
	}
	if (e->format_allocated)
		kfree(e->format);
	kfree(e);
	return 0;
}

static int marker_set_format(struct marker_entry *entry, const char *format)
{
	entry->format = kstrdup(format, GFP_KERNEL);
	if (!entry->format)
		return -ENOMEM;
	entry->format_allocated = 1;

	trace_mark(metadata, core_marker_format,
		   "channel %s name %s format %s",
		   entry->channel, entry->name, entry->format);
	return 0;
}

static int set_marker(struct marker_entry *entry, struct marker *elem,
		int active)
{
	int ret = 0;
	WARN_ON(strcmp(entry->name, elem->name) != 0);

	if (entry->format) {
		if (strcmp(entry->format, elem->format) != 0) {
			printk(KERN_NOTICE
				"Format mismatch for probe %s "
				"(%s), marker (%s)\n",
				entry->name,
				entry->format,
				elem->format);
			return -EPERM;
		}
	} else {
		ret = marker_set_format(entry, elem->format);
		if (ret)
			return ret;
	}

	/*
	 * probe_cb setup (statically known) is done here. It is
	 * asynchronous with the rest of execution, therefore we only
	 * pass from a "safe" callback (with argument) to an "unsafe"
	 * callback (does not set arguments).
	 */
	elem->call = entry->call;
	elem->channel_id = entry->channel_id;
	elem->event_id = entry->event_id;
	/*
	 * Sanity check :
	 * We only update the single probe private data when the ptr is
	 * set to a _non_ single probe! (0 -> 1 and N -> 1, N != 1)
	 */
	WARN_ON(elem->single.func != __mark_empty_function
		&& elem->single.probe_private != entry->single.probe_private
		&& !elem->ptype);
	elem->single.probe_private = entry->single.probe_private;
	/*
	 * Make sure the private data is valid when we update the
	 * single probe ptr.
	 */
	smp_wmb();
	elem->single.func = entry->single.func;
	/*
	 * We also make sure that the new probe callbacks array is consistent
	 * before setting a pointer to it.
	 */
	rcu_assign_pointer(elem->multi, entry->multi);
	/*
	 * Update the function or multi probe array pointer before setting the
	 * ptype.
	 */
	smp_wmb();
	elem->ptype = entry->ptype;

	if (elem->tp_name && (active ^ _imv_read(elem->state))) {
		WARN_ON(!elem->tp_cb);
		/*
		 * It is ok to directly call the probe registration because type
		 * checking has been done in the __trace_mark_tp() macro.
		 */

		if (active) {
			/*
			 * try_module_get should always succeed because we hold
			 * lock_module() to get the tp_cb address.
			 */
			ret = try_module_get(__module_text_address(
				(unsigned long)elem->tp_cb));
			BUG_ON(!ret);
			ret = tracepoint_probe_register_noupdate(
				elem->tp_name,
				elem->tp_cb, NULL);
		} else {
			ret = tracepoint_probe_unregister_noupdate(
				elem->tp_name,
				elem->tp_cb, NULL);
			/*
			 * tracepoint_probe_update_all() must be called
			 * before the module containing tp_cb is unloaded.
			 */
			module_put(__module_text_address(
				(unsigned long)elem->tp_cb));
		}
	}
	elem->state__imv = active;

	return ret;
}

static void disable_marker(struct marker *elem)
{
	int ret;

	/* leave "call" as is. It is known statically. */
	if (elem->tp_name && _imv_read(elem->state)) {
		WARN_ON(!elem->tp_cb);
		/*
		 * It is ok to directly call the probe registration because type
		 * checking has been done in the __trace_mark_tp() macro.
		 */
		ret = tracepoint_probe_unregister_noupdate(elem->tp_name,
			elem->tp_cb, NULL);
		WARN_ON(ret);
		/*
		 * tracepoint_probe_update_all() must be called
		 * before the module containing tp_cb is unloaded.
		 */
		module_put(__module_text_address((unsigned long)elem->tp_cb));
	}
	elem->state__imv = 0;
	elem->single.func = __mark_empty_function;
	/* Update the function before setting the ptype */
	smp_wmb();
	elem->ptype = 0;	/* single probe */
	/*
	 * Leave the private data and channel_id/event_id there, because removal
	 * is racy and should be done only after an RCU period. These are never
	 * used until the next initialization anyway.
	 */
}

int is_marker_present(const char *channel, const char *name)
{
	int ret;
	struct marker_iter iter;

	ret = 0;

	marker_iter_reset(&iter);
	marker_iter_start(&iter);
	for (; iter.marker != NULL; marker_iter_next(&iter)) {
		if (!strcmp(iter.marker->channel, channel) &&
		    !strcmp(iter.marker->name, name)) {
			ret = 1;
			goto end;
		}
	}
end:
	marker_iter_stop(&iter);
	return ret;
}
EXPORT_SYMBOL_GPL(is_marker_present);

int _is_marker_enabled(const char *channel, const char *name)
{
	struct marker_entry *entry;

	entry = get_marker(channel, name);

	return entry && !!entry->refcount;
}
EXPORT_SYMBOL_GPL(_is_marker_enabled);

int is_marker_enabled(const char *channel, const char *name)
{
	int ret;

	lock_markers();
	ret = _is_marker_enabled(channel, name);
	unlock_markers();

	return ret;
}
EXPORT_SYMBOL_GPL(is_marker_enabled);

void marker_update_probe_range(struct marker *begin,
	struct marker *end)
{
	struct marker *iter;
	struct marker_entry *mark_entry;

	mutex_lock(&markers_mutex);
	for (iter = begin; iter < end; iter++) {
		mark_entry = get_marker(iter->channel, iter->name);
		if (mark_entry) {
			set_marker(mark_entry, iter, !!mark_entry->refcount);
			/*
			 * ignore error, continue
			 */
		} else {
			disable_marker(iter);
		}
	}
	mutex_unlock(&markers_mutex);
}

void marker_update_probes(void)
{
	/* Core kernel markers */
	marker_update_probe_range(__start___markers, __stop___markers);
	/* Markers in modules. */
	module_update_markers();
	tracepoint_probe_update_all();
	/* Update immediate values */
	core_imv_update();
	module_imv_update();
}

int marker_probe_register(const char *channel, const char *name,
			  const char *format, marker_probe_func *probe,
			  void *probe_private)
{
	struct marker_entry *entry;
	int ret = 0, ret_err;
	struct marker_probe_array *old;
	int first_probe = 0;

	mutex_lock(&markers_mutex);
	entry = get_marker(channel, name);
	if (!entry) {
		first_probe = 1;
		entry = add_marker(channel, name, format);
		if (IS_ERR(entry))
			ret = PTR_ERR(entry);
		if (ret)
			goto end;
		ret = ltt_channels_register(channel);
		if (ret)
			goto error_remove_marker;
		ret = ltt_channels_get_index_from_name(channel);
		if (ret < 0)
			goto error_unregister_channel;
		entry->channel_id = ret;
		ret = ltt_channels_get_event_id(channel, name);
		if (ret < 0)
			goto error_unregister_channel;
		entry->event_id = ret;
		hlist_add_head(&entry->id_list, id_table + hash_32(
				(entry->channel_id << 16) | entry->event_id,
				MARKER_HASH_BITS));
		ret = 0;
		trace_mark(metadata, core_marker_id,
			   "channel %s name %s event_id %hu "
			   "int #1u%zu long #1u%zu pointer #1u%zu "
			   "size_t #1u%zu alignment #1u%u",
			   channel, name, entry->event_id,
			   sizeof(int), sizeof(long), sizeof(void *),
			   sizeof(size_t), ltt_get_alignment());
	} else if (format) {
		if (!entry->format)
			ret = marker_set_format(entry, format);
		else if (strcmp(entry->format, format))
			ret = -EPERM;
		if (ret)
			goto end;
	}

	old = marker_entry_add_probe(entry, probe, probe_private);
	if (IS_ERR(old)) {
		ret = PTR_ERR(old);
		if (first_probe)
			goto error_unregister_channel;
		else
			goto end;
	}
	mutex_unlock(&markers_mutex);

	marker_update_probes();
	if (old)
		call_rcu_sched(&old->rcu, free_old_closure);
	return ret;

error_unregister_channel:
	ret_err = ltt_channels_unregister(channel, 1);
	WARN_ON(ret_err);
error_remove_marker:
	ret_err = remove_marker(channel, name, 0, 0);
	WARN_ON(ret_err);
end:
	mutex_unlock(&markers_mutex);
	marker_update_probes();	/* for compaction on error path */
	return ret;
}
EXPORT_SYMBOL_GPL(marker_probe_register);

int marker_probe_unregister(const char *channel, const char *name,
			    marker_probe_func *probe, void *probe_private)
{
	struct marker_entry *entry;
	struct marker_probe_array *old;
	int ret = 0;

	mutex_lock(&markers_mutex);
	entry = get_marker(channel, name);
	if (!entry) {
		ret = -ENOENT;
		goto end;
	}
	old = marker_entry_remove_probe(entry, probe, probe_private);
	remove_marker(channel, name, 1, 0);	/* Ignore busy error message */
	mutex_unlock(&markers_mutex);

	marker_update_probes();
	if (old)
		call_rcu_sched(&old->rcu, free_old_closure);
	return ret;

end:
	mutex_unlock(&markers_mutex);
	return ret;
}
EXPORT_SYMBOL_GPL(marker_probe_unregister);

static struct marker_entry *
get_marker_from_private_data(marker_probe_func *probe, void *probe_private)
{
	struct marker_entry *entry;
	unsigned int i;
	struct hlist_head *head;
	struct hlist_node *node;

	for (i = 0; i < MARKER_TABLE_SIZE; i++) {
		head = &marker_table[i];
		hlist_for_each_entry(entry, node, head, hlist) {
			if (!entry->ptype) {
				if (entry->single.func == probe
						&& entry->single.probe_private
						== probe_private)
					return entry;
			} else {
				struct marker_probe_array *closure;
				closure = entry->multi;
				for (i = 0; closure->c[i].func; i++) {
					if (closure->c[i].func == probe &&
					    closure->c[i].probe_private
					    == probe_private)
						return entry;
				}
			}
		}
	}
	return NULL;
}

int marker_probe_unregister_private_data(marker_probe_func *probe,
		void *probe_private)
{
	struct marker_entry *entry;
	int ret = 0;
	struct marker_probe_array *old;
	const char *channel = NULL, *name = NULL;

	mutex_lock(&markers_mutex);
	entry = get_marker_from_private_data(probe, probe_private);
	if (!entry) {
		ret = -ENOENT;
		goto unlock;
	}
	old = marker_entry_remove_probe(entry, NULL, probe_private);
	channel = kstrdup(entry->channel, GFP_KERNEL);
	name = kstrdup(entry->name, GFP_KERNEL);
	remove_marker(channel, name, 1, 0);	/* Ignore busy error message */
	mutex_unlock(&markers_mutex);

	marker_update_probes();
	if (old)
		call_rcu_sched(&old->rcu, free_old_closure);
	goto end;

unlock:
	mutex_unlock(&markers_mutex);
end:
	kfree(channel);
	kfree(name);
	return ret;
}
EXPORT_SYMBOL_GPL(marker_probe_unregister_private_data);

void *marker_get_private_data(const char *channel, const char *name,
			      marker_probe_func *probe, int num)
{
	struct hlist_head *head;
	struct hlist_node *node;
	struct marker_entry *e;
	size_t channel_len = strlen(channel) + 1;
	size_t name_len = strlen(name) + 1;
	int i;
	u32 hash;

	hash = jhash(channel, channel_len-1, 0) ^ jhash(name, name_len-1, 0);
	head = &marker_table[hash & ((1 << MARKER_HASH_BITS)-1)];
	hlist_for_each_entry(e, node, head, hlist) {
		if (!strcmp(channel, e->channel) && !strcmp(name, e->name)) {
			if (!e->ptype) {
				if (num == 0 && e->single.func == probe)
					return e->single.probe_private;
			} else {
				struct marker_probe_array *closure;
				int match = 0;
				closure = e->multi;
				for (i = 0; closure->c[i].func; i++) {
					if (closure->c[i].func != probe)
						continue;
					if (match++ == num)
						return closure->c[i].probe_private;
				}
			}
			break;
		}
	}
	return ERR_PTR(-ENOENT);
}
EXPORT_SYMBOL_GPL(marker_get_private_data);

static struct marker_entry *get_entry_from_id(u16 channel_id, u16 event_id)
{
	struct hlist_head *head;
	struct hlist_node *node;
	struct marker_entry *e, *found = NULL;
	u32 hash = hash_32((channel_id << 16) | event_id, MARKER_HASH_BITS);

	mutex_lock(&markers_mutex);
	head = id_table + hash;
	hlist_for_each_entry(e, node, head, id_list) {
		if (e->channel_id == channel_id && e->event_id == event_id) {
			found = e;
			break;
		}
	}
	mutex_unlock(&markers_mutex);
	return found;
}

/* must call when ids/marker_entry are kept alive */
const char *marker_get_name_from_id(u16 channel_id, u16 event_id)
{
	struct marker_entry *e = get_entry_from_id(channel_id, event_id);
	return e ? e->name : NULL;
}
EXPORT_SYMBOL_GPL(marker_get_name_from_id);

const char *marker_get_fmt_from_id(u16 channel_id, u16 event_id)
{
	struct marker_entry *e = get_entry_from_id(channel_id, event_id);
	return e ? e->format : NULL;
}
EXPORT_SYMBOL_GPL(marker_get_fmt_from_id);

void markers_compact_event_ids(void)
{
	struct marker_entry *entry;
	unsigned int i;
	struct hlist_head *head;
	struct hlist_node *node, *next;
	int ret;

	_ltt_channels_reset_event_ids();

	for (i = 0; i < MARKER_TABLE_SIZE; i++) {
		head = &marker_table[i];
		hlist_for_each_entry_safe(entry, node, next, head, hlist) {
			if (!entry->refcount) {
				remove_marker(entry->channel, entry->name,
					      1, 1);
				continue;
			}
			ret = ltt_channels_get_index_from_name(entry->channel);
			WARN_ON(ret < 0);
			entry->channel_id = ret;
			ret = _ltt_channels_get_event_id(entry->channel,
							 entry->name);
			WARN_ON(ret < 0);
			entry->event_id = ret;
		}
	}

	memset(id_table, 0, sizeof(id_table));
	for (i = 0; i < MARKER_TABLE_SIZE; i++) {
		head = &marker_table[i];
		hlist_for_each_entry(entry, node, head, hlist) {
			hlist_add_head(&entry->id_list, id_table + hash_32(
					(entry->channel_id << 16)
					| entry->event_id, MARKER_HASH_BITS));
		}
	}
}

#ifdef CONFIG_MODULES

int marker_get_iter_range(struct marker **marker, struct marker *begin,
	struct marker *end)
{
	if (!*marker && begin != end) {
		*marker = begin;
		return 1;
	}
	if (*marker >= begin && *marker < end)
		return 1;
	return 0;
}
EXPORT_SYMBOL_GPL(marker_get_iter_range);

static void marker_get_iter(struct marker_iter *iter)
{
	int found = 0;

	/* Core kernel markers */
	if (!iter->module) {
		found = marker_get_iter_range(&iter->marker,
				__start___markers, __stop___markers);
		if (found)
			goto end;
	}
	/* Markers in modules. */
	found = module_get_iter_markers(iter);
end:
	if (!found)
		marker_iter_reset(iter);
}

void marker_iter_start(struct marker_iter *iter)
{
	marker_get_iter(iter);
}
EXPORT_SYMBOL_GPL(marker_iter_start);

void marker_iter_next(struct marker_iter *iter)
{
	iter->marker++;
	/*
	 * iter->marker may be invalid because we blindly incremented it.
	 * Make sure it is valid by marshalling on the markers, getting the
	 * markers from following modules if necessary.
	 */
	marker_get_iter(iter);
}
EXPORT_SYMBOL_GPL(marker_iter_next);

void marker_iter_stop(struct marker_iter *iter)
{
}
EXPORT_SYMBOL_GPL(marker_iter_stop);

void marker_iter_reset(struct marker_iter *iter)
{
	iter->module = NULL;
	iter->marker = NULL;
}
EXPORT_SYMBOL_GPL(marker_iter_reset);

int marker_module_notify(struct notifier_block *self,
			 unsigned long val, void *data)
{
	struct module *mod = data;

	switch (val) {
	case MODULE_STATE_COMING:
		marker_update_probe_range(mod->markers,
			mod->markers + mod->num_markers);
		break;
	case MODULE_STATE_GOING:
		marker_update_probe_range(mod->markers,
			mod->markers + mod->num_markers);
		break;
	}
	return 0;
}

struct notifier_block marker_module_nb = {
	.notifier_call = marker_module_notify,
	.priority = 0,
};

static int init_markers(void)
{
	return register_module_notifier(&marker_module_nb);
}
__initcall(init_markers);

#endif /* CONFIG_MODULES */

void ltt_dump_marker_state(struct ltt_trace *trace)
{
	struct marker_entry *entry;
	struct ltt_probe_private_data call_data;
	struct hlist_head *head;
	struct hlist_node *node;
	unsigned int i;

	mutex_lock(&markers_mutex);
	call_data.trace = trace;
	call_data.serializer = NULL;

	for (i = 0; i < MARKER_TABLE_SIZE; i++) {
		head = &marker_table[i];
		hlist_for_each_entry(entry, node, head, hlist) {
			__trace_mark(0, metadata, core_marker_id,
				&call_data,
				"channel %s name %s event_id %hu "
				"int #1u%zu long #1u%zu pointer #1u%zu "
				"size_t #1u%zu alignment #1u%u",
				entry->channel,
				entry->name,
				entry->event_id,
				sizeof(int), sizeof(long),
				sizeof(void *), sizeof(size_t),
				ltt_get_alignment());
			if (entry->format)
				__trace_mark(0, metadata,
					core_marker_format,
					&call_data,
					"channel %s name %s format %s",
					entry->channel,
					entry->name,
					entry->format);
		}
	}
	mutex_unlock(&markers_mutex);
}
EXPORT_SYMBOL_GPL(ltt_dump_marker_state);
