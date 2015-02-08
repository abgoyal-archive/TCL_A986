
/* A generic hash table package.  */


#ifndef HASH_H_
# define HASH_H_

# include <stdio.h>
# include <stdbool.h>

typedef size_t (*Hash_hasher) (const void *, size_t);
typedef bool (*Hash_comparator) (const void *, const void *);
typedef void (*Hash_data_freer) (void *);
typedef bool (*Hash_processor) (void *, void *);

struct hash_entry
  {
    void *data;
    struct hash_entry *next;
  };

struct hash_tuning
  {
    /* This structure is mainly used for `hash_initialize', see the block
       documentation of `hash_reset_tuning' for more complete comments.  */

    float shrink_threshold;	/* ratio of used buckets to trigger a shrink */
    float shrink_factor;	/* ratio of new smaller size to original size */
    float growth_threshold;	/* ratio of used buckets to trigger a growth */
    float growth_factor;	/* ratio of new bigger size to original size */
    bool is_n_buckets;		/* if CANDIDATE really means table size */
  };

typedef struct hash_tuning Hash_tuning;

struct hash_table;

typedef struct hash_table Hash_table;

/* Information and lookup.  */
size_t hash_get_n_buckets (const Hash_table *);
size_t hash_get_n_buckets_used (const Hash_table *);
size_t hash_get_n_entries (const Hash_table *);
size_t hash_get_max_bucket_length (const Hash_table *);
bool hash_table_ok (const Hash_table *);
void hash_print_statistics (const Hash_table *, FILE *);
void *hash_lookup (const Hash_table *, const void *);

/* Walking.  */
void *hash_get_first (const Hash_table *);
void *hash_get_next (const Hash_table *, const void *);
size_t hash_get_entries (const Hash_table *, void **, size_t);
size_t hash_do_for_each (const Hash_table *, Hash_processor, void *);

/* Allocation and clean-up.  */
size_t hash_string (const char *, size_t);
void hash_reset_tuning (Hash_tuning *);
Hash_table *hash_initialize (size_t, const Hash_tuning *,
			     Hash_hasher, Hash_comparator,
			     Hash_data_freer);
void hash_clear (Hash_table *);
void hash_free (Hash_table *);

/* Insertion and deletion.  */
bool hash_rehash (Hash_table *, size_t);
void *hash_insert (Hash_table *, const void *);
void *hash_delete (Hash_table *, const void *);

#endif
