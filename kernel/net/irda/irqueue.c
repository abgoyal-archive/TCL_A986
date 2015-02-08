


#include <linux/module.h>
#include <linux/slab.h>

#include <net/irda/irda.h>
#include <net/irda/irqueue.h>

/************************ QUEUE SUBROUTINES ************************/

#define GET_HASHBIN(x) ( x & HASHBIN_MASK )

static __u32 hash( const char* name)
{
	__u32 h = 0;
	__u32 g;

	while(*name) {
		h = (h<<4) + *name++;
		if ((g = (h & 0xf0000000)))
			h ^=g>>24;
		h &=~g;
	}
	return h;
}

static void enqueue_first(irda_queue_t **queue, irda_queue_t* element)
{

	IRDA_DEBUG( 4, "%s()\n", __func__);

	/*
	 * Check if queue is empty.
	 */
	if ( *queue == NULL ) {
		/*
		 * Queue is empty.  Insert one element into the queue.
		 */
		element->q_next = element->q_prev = *queue = element;

	} else {
		/*
		 * Queue is not empty.  Insert element into front of queue.
		 */
		element->q_next          = (*queue);
		(*queue)->q_prev->q_next = element;
		element->q_prev          = (*queue)->q_prev;
		(*queue)->q_prev         = element;
		(*queue)                 = element;
	}
}


static irda_queue_t *dequeue_first(irda_queue_t **queue)
{
	irda_queue_t *ret;

	IRDA_DEBUG( 4, "dequeue_first()\n");

	/*
	 * Set return value
	 */
	ret =  *queue;

	if ( *queue == NULL ) {
		/*
		 * Queue was empty.
		 */
	} else if ( (*queue)->q_next == *queue ) {
		/*
		 *  Queue only contained a single element. It will now be
		 *  empty.
		 */
		*queue = NULL;
	} else {
		/*
		 * Queue contained several element.  Remove the first one.
		 */
		(*queue)->q_prev->q_next = (*queue)->q_next;
		(*queue)->q_next->q_prev = (*queue)->q_prev;
		*queue = (*queue)->q_next;
	}

	/*
	 * Return the removed entry (or NULL of queue was empty).
	 */
	return ret;
}

static irda_queue_t *dequeue_general(irda_queue_t **queue, irda_queue_t* element)
{
	irda_queue_t *ret;

	IRDA_DEBUG( 4, "dequeue_general()\n");

	/*
	 * Set return value
	 */
	ret =  *queue;

	if ( *queue == NULL ) {
		/*
		 * Queue was empty.
		 */
	} else if ( (*queue)->q_next == *queue ) {
		/*
		 *  Queue only contained a single element. It will now be
		 *  empty.
		 */
		*queue = NULL;

	} else {
		/*
		 *  Remove specific element.
		 */
		element->q_prev->q_next = element->q_next;
		element->q_next->q_prev = element->q_prev;
		if ( (*queue) == element)
			(*queue) = element->q_next;
	}

	/*
	 * Return the removed entry (or NULL of queue was empty).
	 */
	return ret;
}

/************************ HASHBIN MANAGEMENT ************************/

hashbin_t *hashbin_new(int type)
{
	hashbin_t* hashbin;

	/*
	 * Allocate new hashbin
	 */
	hashbin = kzalloc(sizeof(*hashbin), GFP_ATOMIC);
	if (!hashbin)
		return NULL;

	/*
	 * Initialize structure
	 */
	hashbin->hb_type = type;
	hashbin->magic = HB_MAGIC;
	//hashbin->hb_current = NULL;

	/* Make sure all spinlock's are unlocked */
	if ( hashbin->hb_type & HB_LOCK ) {
		spin_lock_init(&hashbin->hb_spinlock);
	}

	return hashbin;
}
EXPORT_SYMBOL(hashbin_new);


#ifdef CONFIG_LOCKDEP
static int hashbin_lock_depth = 0;
#endif
int hashbin_delete( hashbin_t* hashbin, FREE_FUNC free_func)
{
	irda_queue_t* queue;
	unsigned long flags = 0;
	int i;

	IRDA_ASSERT(hashbin != NULL, return -1;);
	IRDA_ASSERT(hashbin->magic == HB_MAGIC, return -1;);

	/* Synchronize */
	if ( hashbin->hb_type & HB_LOCK ) {
		spin_lock_irqsave_nested(&hashbin->hb_spinlock, flags,
					 hashbin_lock_depth++);
	}

	/*
	 *  Free the entries in the hashbin, TODO: use hashbin_clear when
	 *  it has been shown to work
	 */
	for (i = 0; i < HASHBIN_SIZE; i ++ ) {
		queue = dequeue_first((irda_queue_t**) &hashbin->hb_queue[i]);
		while (queue ) {
			if (free_func)
				(*free_func)(queue);
			queue = dequeue_first(
				(irda_queue_t**) &hashbin->hb_queue[i]);
		}
	}

	/* Cleanup local data */
	hashbin->hb_current = NULL;
	hashbin->magic = ~HB_MAGIC;

	/* Release lock */
	if ( hashbin->hb_type & HB_LOCK) {
		spin_unlock_irqrestore(&hashbin->hb_spinlock, flags);
#ifdef CONFIG_LOCKDEP
		hashbin_lock_depth--;
#endif
	}

	/*
	 *  Free the hashbin structure
	 */
	kfree(hashbin);

	return 0;
}
EXPORT_SYMBOL(hashbin_delete);

/********************* HASHBIN LIST OPERATIONS *********************/

void hashbin_insert(hashbin_t* hashbin, irda_queue_t* entry, long hashv,
		    const char* name)
{
	unsigned long flags = 0;
	int bin;

	IRDA_DEBUG( 4, "%s()\n", __func__);

	IRDA_ASSERT( hashbin != NULL, return;);
	IRDA_ASSERT( hashbin->magic == HB_MAGIC, return;);

	/*
	 * Locate hashbin
	 */
	if ( name )
		hashv = hash( name );
	bin = GET_HASHBIN( hashv );

	/* Synchronize */
	if ( hashbin->hb_type & HB_LOCK ) {
		spin_lock_irqsave(&hashbin->hb_spinlock, flags);
	} /* Default is no-lock  */

	/*
	 * Store name and key
	 */
	entry->q_hash = hashv;
	if ( name )
		strlcpy( entry->q_name, name, sizeof(entry->q_name));

	/*
	 * Insert new entry first
	 */
	enqueue_first( (irda_queue_t**) &hashbin->hb_queue[ bin ],
		       entry);
	hashbin->hb_size++;

	/* Release lock */
	if ( hashbin->hb_type & HB_LOCK ) {
		spin_unlock_irqrestore(&hashbin->hb_spinlock, flags);
	} /* Default is no-lock  */
}
EXPORT_SYMBOL(hashbin_insert);

void *hashbin_remove_first( hashbin_t *hashbin)
{
	unsigned long flags = 0;
	irda_queue_t *entry = NULL;

	/* Synchronize */
	if ( hashbin->hb_type & HB_LOCK ) {
		spin_lock_irqsave(&hashbin->hb_spinlock, flags);
	} /* Default is no-lock  */

	entry = hashbin_get_first( hashbin);
	if ( entry != NULL) {
		int	bin;
		long	hashv;
		/*
		 * Locate hashbin
		 */
		hashv = entry->q_hash;
		bin = GET_HASHBIN( hashv );

		/*
		 * Dequeue the entry...
		 */
		dequeue_general( (irda_queue_t**) &hashbin->hb_queue[ bin ],
				 (irda_queue_t*) entry );
		hashbin->hb_size--;
		entry->q_next = NULL;
		entry->q_prev = NULL;

		/*
		 *  Check if this item is the currently selected item, and in
		 *  that case we must reset hb_current
		 */
		if ( entry == hashbin->hb_current)
			hashbin->hb_current = NULL;
	}

	/* Release lock */
	if ( hashbin->hb_type & HB_LOCK ) {
		spin_unlock_irqrestore(&hashbin->hb_spinlock, flags);
	} /* Default is no-lock  */

	return entry;
}


void* hashbin_remove( hashbin_t* hashbin, long hashv, const char* name)
{
	int bin, found = FALSE;
	unsigned long flags = 0;
	irda_queue_t* entry;

	IRDA_DEBUG( 4, "%s()\n", __func__);

	IRDA_ASSERT( hashbin != NULL, return NULL;);
	IRDA_ASSERT( hashbin->magic == HB_MAGIC, return NULL;);

	/*
	 * Locate hashbin
	 */
	if ( name )
		hashv = hash( name );
	bin = GET_HASHBIN( hashv );

	/* Synchronize */
	if ( hashbin->hb_type & HB_LOCK ) {
		spin_lock_irqsave(&hashbin->hb_spinlock, flags);
	} /* Default is no-lock  */

	/*
	 * Search for entry
	 */
	entry = hashbin->hb_queue[ bin ];
	if ( entry ) {
		do {
			/*
			 * Check for key
			 */
			if ( entry->q_hash == hashv ) {
				/*
				 * Name compare too?
				 */
				if ( name ) {
					if ( strcmp( entry->q_name, name) == 0)
					{
						found = TRUE;
						break;
					}
				} else {
					found = TRUE;
					break;
				}
			}
			entry = entry->q_next;
		} while ( entry != hashbin->hb_queue[ bin ] );
	}

	/*
	 * If entry was found, dequeue it
	 */
	if ( found ) {
		dequeue_general( (irda_queue_t**) &hashbin->hb_queue[ bin ],
				 (irda_queue_t*) entry );
		hashbin->hb_size--;

		/*
		 *  Check if this item is the currently selected item, and in
		 *  that case we must reset hb_current
		 */
		if ( entry == hashbin->hb_current)
			hashbin->hb_current = NULL;
	}

	/* Release lock */
	if ( hashbin->hb_type & HB_LOCK ) {
		spin_unlock_irqrestore(&hashbin->hb_spinlock, flags);
	} /* Default is no-lock  */


	/* Return */
	if ( found )
		return entry;
	else
		return NULL;

}
EXPORT_SYMBOL(hashbin_remove);

void* hashbin_remove_this( hashbin_t* hashbin, irda_queue_t* entry)
{
	unsigned long flags = 0;
	int	bin;
	long	hashv;

	IRDA_DEBUG( 4, "%s()\n", __func__);

	IRDA_ASSERT( hashbin != NULL, return NULL;);
	IRDA_ASSERT( hashbin->magic == HB_MAGIC, return NULL;);
	IRDA_ASSERT( entry != NULL, return NULL;);

	/* Synchronize */
	if ( hashbin->hb_type & HB_LOCK ) {
		spin_lock_irqsave(&hashbin->hb_spinlock, flags);
	} /* Default is no-lock  */

	/* Check if valid and not already removed... */
	if((entry->q_next == NULL) || (entry->q_prev == NULL)) {
		entry = NULL;
		goto out;
	}

	/*
	 * Locate hashbin
	 */
	hashv = entry->q_hash;
	bin = GET_HASHBIN( hashv );

	/*
	 * Dequeue the entry...
	 */
	dequeue_general( (irda_queue_t**) &hashbin->hb_queue[ bin ],
			 (irda_queue_t*) entry );
	hashbin->hb_size--;
	entry->q_next = NULL;
	entry->q_prev = NULL;

	/*
	 *  Check if this item is the currently selected item, and in
	 *  that case we must reset hb_current
	 */
	if ( entry == hashbin->hb_current)
		hashbin->hb_current = NULL;
out:
	/* Release lock */
	if ( hashbin->hb_type & HB_LOCK ) {
		spin_unlock_irqrestore(&hashbin->hb_spinlock, flags);
	} /* Default is no-lock  */

	return entry;
}
EXPORT_SYMBOL(hashbin_remove_this);

/*********************** HASHBIN ENUMERATION ***********************/

void* hashbin_find( hashbin_t* hashbin, long hashv, const char* name )
{
	int bin;
	irda_queue_t* entry;

	IRDA_DEBUG( 4, "hashbin_find()\n");

	IRDA_ASSERT( hashbin != NULL, return NULL;);
	IRDA_ASSERT( hashbin->magic == HB_MAGIC, return NULL;);

	/*
	 * Locate hashbin
	 */
	if ( name )
		hashv = hash( name );
	bin = GET_HASHBIN( hashv );

	/*
	 * Search for entry
	 */
	entry = hashbin->hb_queue[ bin];
	if ( entry ) {
		do {
			/*
			 * Check for key
			 */
			if ( entry->q_hash == hashv ) {
				/*
				 * Name compare too?
				 */
				if ( name ) {
					if ( strcmp( entry->q_name, name ) == 0 ) {
						return entry;
					}
				} else {
					return entry;
				}
			}
			entry = entry->q_next;
		} while ( entry != hashbin->hb_queue[ bin ] );
	}

	return NULL;
}
EXPORT_SYMBOL(hashbin_find);

void* hashbin_lock_find( hashbin_t* hashbin, long hashv, const char* name )
{
	unsigned long flags = 0;
	irda_queue_t* entry;

	/* Synchronize */
	spin_lock_irqsave(&hashbin->hb_spinlock, flags);

	/*
	 * Search for entry
	 */
	entry = (irda_queue_t* ) hashbin_find( hashbin, hashv, name );

	/* Release lock */
	spin_unlock_irqrestore(&hashbin->hb_spinlock, flags);

	return entry;
}
EXPORT_SYMBOL(hashbin_lock_find);

void* hashbin_find_next( hashbin_t* hashbin, long hashv, const char* name,
			 void ** pnext)
{
	unsigned long flags = 0;
	irda_queue_t* entry;

	/* Synchronize */
	spin_lock_irqsave(&hashbin->hb_spinlock, flags);

	/*
	 * Search for current entry
	 * This allow to check if the current item is still in the
	 * hashbin or has been removed.
	 */
	entry = (irda_queue_t* ) hashbin_find( hashbin, hashv, name );

	/*
	 * Trick hashbin_get_next() to return what we want
	 */
	if(entry) {
		hashbin->hb_current = entry;
		*pnext = hashbin_get_next( hashbin );
	} else
		*pnext = NULL;

	/* Release lock */
	spin_unlock_irqrestore(&hashbin->hb_spinlock, flags);

	return entry;
}

irda_queue_t *hashbin_get_first( hashbin_t* hashbin)
{
	irda_queue_t *entry;
	int i;

	IRDA_ASSERT( hashbin != NULL, return NULL;);
	IRDA_ASSERT( hashbin->magic == HB_MAGIC, return NULL;);

	if ( hashbin == NULL)
		return NULL;

	for ( i = 0; i < HASHBIN_SIZE; i ++ ) {
		entry = hashbin->hb_queue[ i];
		if ( entry) {
			hashbin->hb_current = entry;
			return entry;
		}
	}
	/*
	 *  Did not find any item in hashbin
	 */
	return NULL;
}
EXPORT_SYMBOL(hashbin_get_first);

irda_queue_t *hashbin_get_next( hashbin_t *hashbin)
{
	irda_queue_t* entry;
	int bin;
	int i;

	IRDA_ASSERT( hashbin != NULL, return NULL;);
	IRDA_ASSERT( hashbin->magic == HB_MAGIC, return NULL;);

	if ( hashbin->hb_current == NULL) {
		IRDA_ASSERT( hashbin->hb_current != NULL, return NULL;);
		return NULL;
	}
	entry = hashbin->hb_current->q_next;
	bin = GET_HASHBIN( entry->q_hash);

	/*
	 *  Make sure that we are not back at the beginning of the queue
	 *  again
	 */
	if ( entry != hashbin->hb_queue[ bin ]) {
		hashbin->hb_current = entry;

		return entry;
	}

	/*
	 *  Check that this is not the last queue in hashbin
	 */
	if ( bin >= HASHBIN_SIZE)
		return NULL;

	/*
	 *  Move to next queue in hashbin
	 */
	bin++;
	for ( i = bin; i < HASHBIN_SIZE; i++ ) {
		entry = hashbin->hb_queue[ i];
		if ( entry) {
			hashbin->hb_current = entry;

			return entry;
		}
	}
	return NULL;
}
EXPORT_SYMBOL(hashbin_get_next);
