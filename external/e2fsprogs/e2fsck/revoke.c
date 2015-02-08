
#ifndef __KERNEL__
#include "jfs_user.h"
#else
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/jbd.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/locks.h>
#include <linux/list.h>
#include <linux/smp_lock.h>
#include <linux/init.h>
#endif

static kmem_cache_t *revoke_record_cache;
static kmem_cache_t *revoke_table_cache;


struct jbd_revoke_record_s 
{
	struct list_head  hash;
	tid_t		  sequence;	/* Used for recovery only */
	unsigned long	  blocknr;	
};


/* The revoke table is just a simple hash table of revoke records. */
struct jbd_revoke_table_s
{
	/* It is conceivable that we might want a larger hash table
	 * for recovery.  Must be a power of two. */
	int		  hash_size; 
	int		  hash_shift; 
	struct list_head *hash_table;
};


#ifdef __KERNEL__
static void write_one_revoke_record(journal_t *, transaction_t *,
				    struct journal_head **, int *,
				    struct jbd_revoke_record_s *);
static void flush_descriptor(journal_t *, struct journal_head *, int);
#endif

/* Utility functions to maintain the revoke table */

/* Borrowed from buffer.c: this is a tried and tested block hash function */
static inline int hash(journal_t *journal, unsigned long block)
{
	struct jbd_revoke_table_s *table = journal->j_revoke;
	int hash_shift = table->hash_shift;
	
	return ((block << (hash_shift - 6)) ^
		(block >> 13) ^
		(block << (hash_shift - 12))) & (table->hash_size - 1);
}

static int insert_revoke_hash(journal_t *journal, unsigned long blocknr,
			      tid_t seq)
{
	struct list_head *hash_list;
	struct jbd_revoke_record_s *record;

#ifdef __KERNEL__
repeat:
#endif
	record = kmem_cache_alloc(revoke_record_cache, GFP_NOFS);
	if (!record)
		goto oom;

	record->sequence = seq;
	record->blocknr = blocknr;
	hash_list = &journal->j_revoke->hash_table[hash(journal, blocknr)];
	list_add(&record->hash, hash_list);
	return 0;

oom:
#ifdef __KERNEL__
	if (!journal_oom_retry)
		return -ENOMEM;
	jbd_debug(1, "ENOMEM in " __FUNCTION__ ", retrying.\n");
	current->policy |= SCHED_YIELD;
	schedule();
	goto repeat;
#else
	return -ENOMEM;
#endif
}

/* Find a revoke record in the journal's hash table. */

static struct jbd_revoke_record_s *find_revoke_record(journal_t *journal,
						      unsigned long blocknr)
{
	struct list_head *hash_list;
	struct jbd_revoke_record_s *record;
	
	hash_list = &journal->j_revoke->hash_table[hash(journal, blocknr)];

	record = (struct jbd_revoke_record_s *) hash_list->next;
	while (&(record->hash) != hash_list) {
		if (record->blocknr == blocknr)
			return record;
		record = (struct jbd_revoke_record_s *) record->hash.next;
	}
	return NULL;
}

int __init journal_init_revoke_caches(void)
{
	revoke_record_cache = kmem_cache_create("revoke_record",
					   sizeof(struct jbd_revoke_record_s),
					   0, SLAB_HWCACHE_ALIGN, NULL, NULL);
	if (revoke_record_cache == 0)
		return -ENOMEM;

	revoke_table_cache = kmem_cache_create("revoke_table",
					   sizeof(struct jbd_revoke_table_s),
					   0, 0, NULL, NULL);
	if (revoke_table_cache == 0) {
		kmem_cache_destroy(revoke_record_cache);
		revoke_record_cache = NULL;
		return -ENOMEM;
	}
	return 0;
}	

void journal_destroy_revoke_caches(void)
{
	kmem_cache_destroy(revoke_record_cache);
	revoke_record_cache = 0;
	kmem_cache_destroy(revoke_table_cache);
	revoke_table_cache = 0;
}

/* Initialise the revoke table for a given journal to a given size. */

int journal_init_revoke(journal_t *journal, int hash_size)
{
	int shift, tmp;
	
	J_ASSERT (journal->j_revoke == NULL);
	
	journal->j_revoke = kmem_cache_alloc(revoke_table_cache, GFP_KERNEL);
	if (!journal->j_revoke)
		return -ENOMEM;
	
	/* Check that the hash_size is a power of two */
	J_ASSERT ((hash_size & (hash_size-1)) == 0);

	journal->j_revoke->hash_size = hash_size;

	shift = 0;
	tmp = hash_size;
	while((tmp >>= 1UL) != 0UL)
		shift++;
	journal->j_revoke->hash_shift = shift;

	journal->j_revoke->hash_table =
		kmalloc(hash_size * sizeof(struct list_head), GFP_KERNEL);
	if (!journal->j_revoke->hash_table) {
		kmem_cache_free(revoke_table_cache, journal->j_revoke);
		journal->j_revoke = NULL;
		return -ENOMEM;
	}
	
	for (tmp = 0; tmp < hash_size; tmp++)
		INIT_LIST_HEAD(&journal->j_revoke->hash_table[tmp]);
	
	return 0;
}

/* Destoy a journal's revoke table.  The table must already be empty! */

void journal_destroy_revoke(journal_t *journal)
{
	struct jbd_revoke_table_s *table;
	struct list_head *hash_list;
	int i;
	
	table = journal->j_revoke;
	if (!table)
		return;
	
	for (i=0; i<table->hash_size; i++) {
		hash_list = &table->hash_table[i];
		J_ASSERT (list_empty(hash_list));
	}
	
	kfree(table->hash_table);
	kmem_cache_free(revoke_table_cache, table);
	journal->j_revoke = NULL;
}


#ifdef __KERNEL__


int journal_revoke(handle_t *handle, unsigned long blocknr, 
		   struct buffer_head *bh_in)
{
	struct buffer_head *bh = NULL;
	journal_t *journal;
	kdev_t dev;
	int err;

	if (bh_in)
		BUFFER_TRACE(bh_in, "enter");

	journal = handle->h_transaction->t_journal;
	if (!journal_set_features(journal, 0, 0, JFS_FEATURE_INCOMPAT_REVOKE)){
		J_ASSERT (!"Cannot set revoke feature!");
		return -EINVAL;
	}

	dev = journal->j_fs_dev;
	bh = bh_in;

	if (!bh) {
		bh = get_hash_table(dev, blocknr, journal->j_blocksize);
		if (bh)
			BUFFER_TRACE(bh, "found on hash");
	}
#ifdef JBD_EXPENSIVE_CHECKING
	else {
		struct buffer_head *bh2;

		/* If there is a different buffer_head lying around in
		 * memory anywhere... */
		bh2 = get_hash_table(dev, blocknr, journal->j_blocksize);
		if (bh2) {
			/* ... and it has RevokeValid status... */
			if ((bh2 != bh) &&
			    test_bit(BH_RevokeValid, &bh2->b_state))
				/* ...then it better be revoked too,
				 * since it's illegal to create a revoke
				 * record against a buffer_head which is
				 * not marked revoked --- that would
				 * risk missing a subsequent revoke
				 * cancel. */
				J_ASSERT_BH(bh2, test_bit(BH_Revoked, &
							  bh2->b_state));
			__brelse(bh2);
		}
	}
#endif

	/* We really ought not ever to revoke twice in a row without
           first having the revoke cancelled: it's illegal to free a
           block twice without allocating it in between! */
	if (bh) {
		J_ASSERT_BH(bh, !test_bit(BH_Revoked, &bh->b_state));
		set_bit(BH_Revoked, &bh->b_state);
		set_bit(BH_RevokeValid, &bh->b_state);
		if (bh_in) {
			BUFFER_TRACE(bh_in, "call journal_forget");
			journal_forget(handle, bh_in);
		} else {
			BUFFER_TRACE(bh, "call brelse");
			__brelse(bh);
		}
	}

	lock_journal(journal);
	jbd_debug(2, "insert revoke for block %lu, bh_in=%p\n", blocknr, bh_in);
	err = insert_revoke_hash(journal, blocknr,
				handle->h_transaction->t_tid);
	unlock_journal(journal);
	BUFFER_TRACE(bh_in, "exit");
	return err;
}

int journal_cancel_revoke(handle_t *handle, struct journal_head *jh)
{
	struct jbd_revoke_record_s *record;
	journal_t *journal = handle->h_transaction->t_journal;
	int need_cancel;
	int did_revoke = 0;	/* akpm: debug */
	struct buffer_head *bh = jh2bh(jh);
	
	jbd_debug(4, "journal_head %p, cancelling revoke\n", jh);

	/* Is the existing Revoke bit valid?  If so, we trust it, and
	 * only perform the full cancel if the revoke bit is set.  If
	 * not, we can't trust the revoke bit, and we need to do the
	 * full search for a revoke record. */
	if (test_and_set_bit(BH_RevokeValid, &bh->b_state))
		need_cancel = (test_and_clear_bit(BH_Revoked, &bh->b_state));
	else {
		need_cancel = 1;
		clear_bit(BH_Revoked, &bh->b_state);
	}

	if (need_cancel) {
		record = find_revoke_record(journal, bh->b_blocknr);
		if (record) {
			jbd_debug(4, "cancelled existing revoke on "
				  "blocknr %lu\n", bh->b_blocknr);
			list_del(&record->hash);
			kmem_cache_free(revoke_record_cache, record);
			did_revoke = 1;
		}
	}

#ifdef JBD_EXPENSIVE_CHECKING
	/* There better not be one left behind by now! */
	record = find_revoke_record(journal, bh->b_blocknr);
	J_ASSERT_JH(jh, record == NULL);
#endif

	/* Finally, have we just cleared revoke on an unhashed
	 * buffer_head?  If so, we'd better make sure we clear the
	 * revoked status on any hashed alias too, otherwise the revoke
	 * state machine will get very upset later on. */
	if (need_cancel && !bh->b_pprev) {
		struct buffer_head *bh2;
		bh2 = get_hash_table(bh->b_dev, bh->b_blocknr, bh->b_size);
		if (bh2) {
			clear_bit(BH_Revoked, &bh2->b_state);
			__brelse(bh2);
		}
	}
	
	return did_revoke;
}



void journal_write_revoke_records(journal_t *journal, 
				  transaction_t *transaction)
{
	struct journal_head *descriptor;
	struct jbd_revoke_record_s *record;
	struct jbd_revoke_table_s *revoke;
	struct list_head *hash_list;
	int i, offset, count;

	descriptor = NULL; 
	offset = 0;
	count = 0;
	revoke = journal->j_revoke;
	
	for (i = 0; i < revoke->hash_size; i++) {
		hash_list = &revoke->hash_table[i];

		while (!list_empty(hash_list)) {
			record = (struct jbd_revoke_record_s *) 
				hash_list->next;
			write_one_revoke_record(journal, transaction,
						&descriptor, &offset, 
						record);
			count++;
			list_del(&record->hash);
			kmem_cache_free(revoke_record_cache, record);
		}
	}
	if (descriptor) 
		flush_descriptor(journal, descriptor, offset);
	jbd_debug(1, "Wrote %d revoke records\n", count);
}


static void write_one_revoke_record(journal_t *journal, 
				    transaction_t *transaction,
				    struct journal_head **descriptorp, 
				    int *offsetp,
				    struct jbd_revoke_record_s *record)
{
	struct journal_head *descriptor;
	int offset;
	journal_header_t *header;

	/* If we are already aborting, this all becomes a noop.  We
           still need to go round the loop in
           journal_write_revoke_records in order to free all of the
           revoke records: only the IO to the journal is omitted. */
	if (is_journal_aborted(journal))
		return;

	descriptor = *descriptorp;
	offset = *offsetp;

	/* Make sure we have a descriptor with space left for the record */
	if (descriptor) {
		if (offset == journal->j_blocksize) {
			flush_descriptor(journal, descriptor, offset);
			descriptor = NULL;
		}
	}
	
	if (!descriptor) {
		descriptor = journal_get_descriptor_buffer(journal);
		if (!descriptor)
			return;
		header = (journal_header_t *) &jh2bh(descriptor)->b_data[0];
		header->h_magic     = htonl(JFS_MAGIC_NUMBER);
		header->h_blocktype = htonl(JFS_REVOKE_BLOCK);
		header->h_sequence  = htonl(transaction->t_tid);

		/* Record it so that we can wait for IO completion later */
		JBUFFER_TRACE(descriptor, "file as BJ_LogCtl");
		journal_file_buffer(descriptor, transaction, BJ_LogCtl);

		offset = sizeof(journal_revoke_header_t);
		*descriptorp = descriptor;
	}
	
	* ((unsigned int *)(&jh2bh(descriptor)->b_data[offset])) = 
		htonl(record->blocknr);
	offset += 4;
	*offsetp = offset;
}


static void flush_descriptor(journal_t *journal, 
			     struct journal_head *descriptor, 
			     int offset)
{
	journal_revoke_header_t *header;

	if (is_journal_aborted(journal)) {
		JBUFFER_TRACE(descriptor, "brelse");
		__brelse(jh2bh(descriptor));
		return;
	}
	
	header = (journal_revoke_header_t *) jh2bh(descriptor)->b_data;
	header->r_count = htonl(offset);
	set_bit(BH_JWrite, &jh2bh(descriptor)->b_state);
	{
		struct buffer_head *bh = jh2bh(descriptor);
		BUFFER_TRACE(bh, "write");
		ll_rw_block (WRITE, 1, &bh);
	}
}

#endif



int journal_set_revoke(journal_t *journal, 
		       unsigned long blocknr, 
		       tid_t sequence)
{
	struct jbd_revoke_record_s *record;
	
	record = find_revoke_record(journal, blocknr);
	if (record) {
		/* If we have multiple occurences, only record the
		 * latest sequence number in the hashed record */
		if (tid_gt(sequence, record->sequence))
			record->sequence = sequence;
		return 0;
	} 
	return insert_revoke_hash(journal, blocknr, sequence);
}


int journal_test_revoke(journal_t *journal, 
			unsigned long blocknr,
			tid_t sequence)
{
	struct jbd_revoke_record_s *record;
	
	record = find_revoke_record(journal, blocknr);
	if (!record)
		return 0;
	if (tid_gt(sequence, record->sequence))
		return 0;
	return 1;
}


void journal_clear_revoke(journal_t *journal)
{
	int i;
	struct list_head *hash_list;
	struct jbd_revoke_record_s *record;
	struct jbd_revoke_table_s *revoke;
	
	revoke = journal->j_revoke;
	
	for (i = 0; i < revoke->hash_size; i++) {
		hash_list = &revoke->hash_table[i];
		while (!list_empty(hash_list)) {
			record = (struct jbd_revoke_record_s*) hash_list->next;
			list_del(&record->hash);
			kmem_cache_free(revoke_record_cache, record);
		}
	}
}

