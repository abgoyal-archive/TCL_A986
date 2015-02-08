

#include "core.h"
#include "ref.h"


struct reference {
	void *object;
	spinlock_t lock;
	u32 ref;
};


struct ref_table {
	struct reference *entries;
	u32 capacity;
	u32 init_point;
	u32 first_free;
	u32 last_free;
	u32 index_mask;
	u32 start_mask;
};


static struct ref_table tipc_ref_table = { NULL };

static DEFINE_RWLOCK(ref_table_lock);


int tipc_ref_table_init(u32 requested_size, u32 start)
{
	struct reference *table;
	u32 actual_size;

	/* account for unused entry, then round up size to a power of 2 */

	requested_size++;
	for (actual_size = 16; actual_size < requested_size; actual_size <<= 1)
		/* do nothing */ ;

	/* allocate table & mark all entries as uninitialized */

	table = __vmalloc(actual_size * sizeof(struct reference),
			  GFP_KERNEL | __GFP_HIGHMEM | __GFP_ZERO, PAGE_KERNEL);
	if (table == NULL)
		return -ENOMEM;

	tipc_ref_table.entries = table;
	tipc_ref_table.capacity = requested_size;
	tipc_ref_table.init_point = 1;
	tipc_ref_table.first_free = 0;
	tipc_ref_table.last_free = 0;
	tipc_ref_table.index_mask = actual_size - 1;
	tipc_ref_table.start_mask = start & ~tipc_ref_table.index_mask;

	return 0;
}


void tipc_ref_table_stop(void)
{
	if (!tipc_ref_table.entries)
		return;

	vfree(tipc_ref_table.entries);
	tipc_ref_table.entries = NULL;
}


u32 tipc_ref_acquire(void *object, spinlock_t **lock)
{
	u32 index;
	u32 index_mask;
	u32 next_plus_upper;
	u32 ref;
	struct reference *entry = NULL;

	if (!object) {
		err("Attempt to acquire reference to non-existent object\n");
		return 0;
	}
	if (!tipc_ref_table.entries) {
		err("Reference table not found during acquisition attempt\n");
		return 0;
	}

	/* take a free entry, if available; otherwise initialize a new entry */

	write_lock_bh(&ref_table_lock);
	if (tipc_ref_table.first_free) {
		index = tipc_ref_table.first_free;
		entry = &(tipc_ref_table.entries[index]);
		index_mask = tipc_ref_table.index_mask;
		next_plus_upper = entry->ref;
		tipc_ref_table.first_free = next_plus_upper & index_mask;
		ref = (next_plus_upper & ~index_mask) + index;
	}
	else if (tipc_ref_table.init_point < tipc_ref_table.capacity) {
		index = tipc_ref_table.init_point++;
		entry = &(tipc_ref_table.entries[index]);
		spin_lock_init(&entry->lock);
		ref = tipc_ref_table.start_mask + index;
	}
	else {
		ref = 0;
	}
	write_unlock_bh(&ref_table_lock);

	/*
	 * Grab the lock so no one else can modify this entry
	 * While we assign its ref value & object pointer
	 */
	if (entry) {
		spin_lock_bh(&entry->lock);
		entry->ref = ref;
		entry->object = object;
		*lock = &entry->lock;
		/*
		 * keep it locked, the caller is responsible
		 * for unlocking this when they're done with it
		 */
	}

	return ref;
}


void tipc_ref_discard(u32 ref)
{
	struct reference *entry;
	u32 index;
	u32 index_mask;

	if (!tipc_ref_table.entries) {
		err("Reference table not found during discard attempt\n");
		return;
	}

	index_mask = tipc_ref_table.index_mask;
	index = ref & index_mask;
	entry = &(tipc_ref_table.entries[index]);

	write_lock_bh(&ref_table_lock);

	if (!entry->object) {
		err("Attempt to discard reference to non-existent object\n");
		goto exit;
	}
	if (entry->ref != ref) {
		err("Attempt to discard non-existent reference\n");
		goto exit;
	}

	/*
	 * mark entry as unused; increment instance part of entry's reference
	 * to invalidate any subsequent references
	 */

	entry->object = NULL;
	entry->ref = (ref & ~index_mask) + (index_mask + 1);

	/* append entry to free entry list */

	if (tipc_ref_table.first_free == 0)
		tipc_ref_table.first_free = index;
	else
		tipc_ref_table.entries[tipc_ref_table.last_free].ref |= index;
	tipc_ref_table.last_free = index;

exit:
	write_unlock_bh(&ref_table_lock);
}


void *tipc_ref_lock(u32 ref)
{
	if (likely(tipc_ref_table.entries)) {
		struct reference *entry;

		entry = &tipc_ref_table.entries[ref &
						tipc_ref_table.index_mask];
		if (likely(entry->ref != 0)) {
			spin_lock_bh(&entry->lock);
			if (likely((entry->ref == ref) && (entry->object)))
				return entry->object;
			spin_unlock_bh(&entry->lock);
		}
	}
	return NULL;
}


void tipc_ref_unlock(u32 ref)
{
	if (likely(tipc_ref_table.entries)) {
		struct reference *entry;

		entry = &tipc_ref_table.entries[ref &
						tipc_ref_table.index_mask];
		if (likely((entry->ref == ref) && (entry->object)))
			spin_unlock_bh(&entry->lock);
		else
			err("Attempt to unlock non-existent reference\n");
	}
}


void *tipc_ref_deref(u32 ref)
{
	if (likely(tipc_ref_table.entries)) {
		struct reference *entry;

		entry = &tipc_ref_table.entries[ref &
						tipc_ref_table.index_mask];
		if (likely(entry->ref == ref))
			return entry->object;
	}
	return NULL;
}

