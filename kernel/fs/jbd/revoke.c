

#ifndef __KERNEL__
#include "jfs_user.h"
#else
#include <linux/time.h>
#include <linux/fs.h>
#include <linux/jbd.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/init.h>
#include <linux/bio.h>
#endif
#include <linux/log2.h>

static struct kmem_cache *revoke_record_cache;
static struct kmem_cache *revoke_table_cache;


struct jbd_revoke_record_s
{
	struct list_head  hash;
	tid_t		  sequence;	/* Used for recovery only */
	unsigned int	  blocknr;
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
				    struct jbd_revoke_record_s *, int);
static void flush_descriptor(journal_t *, struct journal_head *, int, int);
#endif

/* Utility functions to maintain the revoke table */

/* Borrowed from buffer.c: this is a tried and tested block hash function */
static inline int hash(journal_t *journal, unsigned int block)
{
	struct jbd_revoke_table_s *table = journal->j_revoke;
	int hash_shift = table->hash_shift;

	return ((block << (hash_shift - 6)) ^
		(block >> 13) ^
		(block << (hash_shift - 12))) & (table->hash_size - 1);
}

static int insert_revoke_hash(journal_t *journal, unsigned int blocknr,
			      tid_t seq)
{
	struct list_head *hash_list;
	struct jbd_revoke_record_s *record;

repeat:
	record = kmem_cache_alloc(revoke_record_cache, GFP_NOFS);
	if (!record)
		goto oom;

	record->sequence = seq;
	record->blocknr = blocknr;
	hash_list = &journal->j_revoke->hash_table[hash(journal, blocknr)];
	spin_lock(&journal->j_revoke_lock);
	list_add(&record->hash, hash_list);
	spin_unlock(&journal->j_revoke_lock);
	return 0;

oom:
	if (!journal_oom_retry)
		return -ENOMEM;
	jbd_debug(1, "ENOMEM in %s, retrying\n", __func__);
	yield();
	goto repeat;
}

/* Find a revoke record in the journal's hash table. */

static struct jbd_revoke_record_s *find_revoke_record(journal_t *journal,
						      unsigned int blocknr)
{
	struct list_head *hash_list;
	struct jbd_revoke_record_s *record;

	hash_list = &journal->j_revoke->hash_table[hash(journal, blocknr)];

	spin_lock(&journal->j_revoke_lock);
	record = (struct jbd_revoke_record_s *) hash_list->next;
	while (&(record->hash) != hash_list) {
		if (record->blocknr == blocknr) {
			spin_unlock(&journal->j_revoke_lock);
			return record;
		}
		record = (struct jbd_revoke_record_s *) record->hash.next;
	}
	spin_unlock(&journal->j_revoke_lock);
	return NULL;
}

void journal_destroy_revoke_caches(void)
{
	if (revoke_record_cache) {
		kmem_cache_destroy(revoke_record_cache);
		revoke_record_cache = NULL;
	}
	if (revoke_table_cache) {
		kmem_cache_destroy(revoke_table_cache);
		revoke_table_cache = NULL;
	}
}

int __init journal_init_revoke_caches(void)
{
	J_ASSERT(!revoke_record_cache);
	J_ASSERT(!revoke_table_cache);

	revoke_record_cache = kmem_cache_create("revoke_record",
					   sizeof(struct jbd_revoke_record_s),
					   0,
					   SLAB_HWCACHE_ALIGN|SLAB_TEMPORARY,
					   NULL);
	if (!revoke_record_cache)
		goto record_cache_failure;

	revoke_table_cache = kmem_cache_create("revoke_table",
					   sizeof(struct jbd_revoke_table_s),
					   0, SLAB_TEMPORARY, NULL);
	if (!revoke_table_cache)
		goto table_cache_failure;

	return 0;

table_cache_failure:
	journal_destroy_revoke_caches();
record_cache_failure:
	return -ENOMEM;
}

static struct jbd_revoke_table_s *journal_init_revoke_table(int hash_size)
{
	int shift = 0;
	int tmp = hash_size;
	struct jbd_revoke_table_s *table;

	table = kmem_cache_alloc(revoke_table_cache, GFP_KERNEL);
	if (!table)
		goto out;

	while((tmp >>= 1UL) != 0UL)
		shift++;

	table->hash_size = hash_size;
	table->hash_shift = shift;
	table->hash_table =
		kmalloc(hash_size * sizeof(struct list_head), GFP_KERNEL);
	if (!table->hash_table) {
		kmem_cache_free(revoke_table_cache, table);
		table = NULL;
		goto out;
	}

	for (tmp = 0; tmp < hash_size; tmp++)
		INIT_LIST_HEAD(&table->hash_table[tmp]);

out:
	return table;
}

static void journal_destroy_revoke_table(struct jbd_revoke_table_s *table)
{
	int i;
	struct list_head *hash_list;

	for (i = 0; i < table->hash_size; i++) {
		hash_list = &table->hash_table[i];
		J_ASSERT(list_empty(hash_list));
	}

	kfree(table->hash_table);
	kmem_cache_free(revoke_table_cache, table);
}

/* Initialise the revoke table for a given journal to a given size. */
int journal_init_revoke(journal_t *journal, int hash_size)
{
	J_ASSERT(journal->j_revoke_table[0] == NULL);
	J_ASSERT(is_power_of_2(hash_size));

	journal->j_revoke_table[0] = journal_init_revoke_table(hash_size);
	if (!journal->j_revoke_table[0])
		goto fail0;

	journal->j_revoke_table[1] = journal_init_revoke_table(hash_size);
	if (!journal->j_revoke_table[1])
		goto fail1;

	journal->j_revoke = journal->j_revoke_table[1];

	spin_lock_init(&journal->j_revoke_lock);

	return 0;

fail1:
	journal_destroy_revoke_table(journal->j_revoke_table[0]);
fail0:
	return -ENOMEM;
}

/* Destroy a journal's revoke table.  The table must already be empty! */
void journal_destroy_revoke(journal_t *journal)
{
	journal->j_revoke = NULL;
	if (journal->j_revoke_table[0])
		journal_destroy_revoke_table(journal->j_revoke_table[0]);
	if (journal->j_revoke_table[1])
		journal_destroy_revoke_table(journal->j_revoke_table[1]);
}


#ifdef __KERNEL__


int journal_revoke(handle_t *handle, unsigned int blocknr,
		   struct buffer_head *bh_in)
{
	struct buffer_head *bh = NULL;
	journal_t *journal;
	struct block_device *bdev;
	int err;

	might_sleep();
	if (bh_in)
		BUFFER_TRACE(bh_in, "enter");

	journal = handle->h_transaction->t_journal;
	if (!journal_set_features(journal, 0, 0, JFS_FEATURE_INCOMPAT_REVOKE)){
		J_ASSERT (!"Cannot set revoke feature!");
		return -EINVAL;
	}

	bdev = journal->j_fs_dev;
	bh = bh_in;

	if (!bh) {
		bh = __find_get_block(bdev, blocknr, journal->j_blocksize);
		if (bh)
			BUFFER_TRACE(bh, "found on hash");
	}
#ifdef JBD_EXPENSIVE_CHECKING
	else {
		struct buffer_head *bh2;

		/* If there is a different buffer_head lying around in
		 * memory anywhere... */
		bh2 = __find_get_block(bdev, blocknr, journal->j_blocksize);
		if (bh2) {
			/* ... and it has RevokeValid status... */
			if (bh2 != bh && buffer_revokevalid(bh2))
				/* ...then it better be revoked too,
				 * since it's illegal to create a revoke
				 * record against a buffer_head which is
				 * not marked revoked --- that would
				 * risk missing a subsequent revoke
				 * cancel. */
				J_ASSERT_BH(bh2, buffer_revoked(bh2));
			put_bh(bh2);
		}
	}
#endif

	/* We really ought not ever to revoke twice in a row without
           first having the revoke cancelled: it's illegal to free a
           block twice without allocating it in between! */
	if (bh) {
		if (!J_EXPECT_BH(bh, !buffer_revoked(bh),
				 "inconsistent data on disk")) {
			if (!bh_in)
				brelse(bh);
			return -EIO;
		}
		set_buffer_revoked(bh);
		set_buffer_revokevalid(bh);
		if (bh_in) {
			BUFFER_TRACE(bh_in, "call journal_forget");
			journal_forget(handle, bh_in);
		} else {
			BUFFER_TRACE(bh, "call brelse");
			__brelse(bh);
		}
	}

	jbd_debug(2, "insert revoke for block %u, bh_in=%p\n", blocknr, bh_in);
	err = insert_revoke_hash(journal, blocknr,
				handle->h_transaction->t_tid);
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
	if (test_set_buffer_revokevalid(bh)) {
		need_cancel = test_clear_buffer_revoked(bh);
	} else {
		need_cancel = 1;
		clear_buffer_revoked(bh);
	}

	if (need_cancel) {
		record = find_revoke_record(journal, bh->b_blocknr);
		if (record) {
			jbd_debug(4, "cancelled existing revoke on "
				  "blocknr %llu\n", (unsigned long long)bh->b_blocknr);
			spin_lock(&journal->j_revoke_lock);
			list_del(&record->hash);
			spin_unlock(&journal->j_revoke_lock);
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
	if (need_cancel) {
		struct buffer_head *bh2;
		bh2 = __find_get_block(bh->b_bdev, bh->b_blocknr, bh->b_size);
		if (bh2) {
			if (bh2 != bh)
				clear_buffer_revoked(bh2);
			__brelse(bh2);
		}
	}
	return did_revoke;
}

void journal_switch_revoke_table(journal_t *journal)
{
	int i;

	if (journal->j_revoke == journal->j_revoke_table[0])
		journal->j_revoke = journal->j_revoke_table[1];
	else
		journal->j_revoke = journal->j_revoke_table[0];

	for (i = 0; i < journal->j_revoke->hash_size; i++)
		INIT_LIST_HEAD(&journal->j_revoke->hash_table[i]);
}

void journal_write_revoke_records(journal_t *journal,
				  transaction_t *transaction, int write_op)
{
	struct journal_head *descriptor;
	struct jbd_revoke_record_s *record;
	struct jbd_revoke_table_s *revoke;
	struct list_head *hash_list;
	int i, offset, count;

	descriptor = NULL;
	offset = 0;
	count = 0;

	/* select revoke table for committing transaction */
	revoke = journal->j_revoke == journal->j_revoke_table[0] ?
		journal->j_revoke_table[1] : journal->j_revoke_table[0];

	for (i = 0; i < revoke->hash_size; i++) {
		hash_list = &revoke->hash_table[i];

		while (!list_empty(hash_list)) {
			record = (struct jbd_revoke_record_s *)
				hash_list->next;
			write_one_revoke_record(journal, transaction,
						&descriptor, &offset,
						record, write_op);
			count++;
			list_del(&record->hash);
			kmem_cache_free(revoke_record_cache, record);
		}
	}
	if (descriptor)
		flush_descriptor(journal, descriptor, offset, write_op);
	jbd_debug(1, "Wrote %d revoke records\n", count);
}


static void write_one_revoke_record(journal_t *journal,
				    transaction_t *transaction,
				    struct journal_head **descriptorp,
				    int *offsetp,
				    struct jbd_revoke_record_s *record,
				    int write_op)
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
			flush_descriptor(journal, descriptor, offset, write_op);
			descriptor = NULL;
		}
	}

	if (!descriptor) {
		descriptor = journal_get_descriptor_buffer(journal);
		if (!descriptor)
			return;
		header = (journal_header_t *) &jh2bh(descriptor)->b_data[0];
		header->h_magic     = cpu_to_be32(JFS_MAGIC_NUMBER);
		header->h_blocktype = cpu_to_be32(JFS_REVOKE_BLOCK);
		header->h_sequence  = cpu_to_be32(transaction->t_tid);

		/* Record it so that we can wait for IO completion later */
		JBUFFER_TRACE(descriptor, "file as BJ_LogCtl");
		journal_file_buffer(descriptor, transaction, BJ_LogCtl);

		offset = sizeof(journal_revoke_header_t);
		*descriptorp = descriptor;
	}

	* ((__be32 *)(&jh2bh(descriptor)->b_data[offset])) =
		cpu_to_be32(record->blocknr);
	offset += 4;
	*offsetp = offset;
}


static void flush_descriptor(journal_t *journal,
			     struct journal_head *descriptor,
			     int offset, int write_op)
{
	journal_revoke_header_t *header;
	struct buffer_head *bh = jh2bh(descriptor);

	if (is_journal_aborted(journal)) {
		put_bh(bh);
		return;
	}

	header = (journal_revoke_header_t *) jh2bh(descriptor)->b_data;
	header->r_count = cpu_to_be32(offset);
	set_buffer_jwrite(bh);
	BUFFER_TRACE(bh, "write");
	set_buffer_dirty(bh);
	ll_rw_block((write_op == WRITE) ? SWRITE : SWRITE_SYNC_PLUG, 1, &bh);
}
#endif



int journal_set_revoke(journal_t *journal,
		       unsigned int blocknr,
		       tid_t sequence)
{
	struct jbd_revoke_record_s *record;

	record = find_revoke_record(journal, blocknr);
	if (record) {
		/* If we have multiple occurrences, only record the
		 * latest sequence number in the hashed record */
		if (tid_gt(sequence, record->sequence))
			record->sequence = sequence;
		return 0;
	}
	return insert_revoke_hash(journal, blocknr, sequence);
}


int journal_test_revoke(journal_t *journal,
			unsigned int blocknr,
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
