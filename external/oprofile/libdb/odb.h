
#ifndef ODB_HASH_H
#define ODB_HASH_H

#include <stddef.h>
#include <stdint.h>

#include "op_list.h"

/** the type of key. 64-bit because CG needs 32-bit pair {from,to} */
typedef uint64_t odb_key_t;
/** the type of an information in the database */
typedef unsigned int odb_value_t;
/** the type of index (node number), list are implemented through index */
typedef unsigned int odb_index_t;
/** the type store node number */
typedef odb_index_t odb_node_nr_t;
/** store the hash mask, hash table size are always power of two */
typedef odb_index_t odb_hash_mask_t;

#define BUCKET_FACTOR 1

/** a db hash node */
typedef struct {
	odb_key_t key;			/**< eip */
	odb_value_t value;		/**< samples count */
	odb_index_t next;		/**< next entry for this bucket */
} odb_node_t;

typedef struct {
	odb_node_nr_t size;		/**< in node nr (power of two) */
	odb_node_nr_t current_size;	/**< nr used node + 1, node 0 unused */
	int padding[6];			/**< for padding and future use */
} odb_descr_t;

typedef struct odb_data {
	odb_node_t * node_base;		/**< base memory area of the page */
	odb_index_t * hash_base;	/**< base memory of hash table */
	odb_descr_t * descr;		/**< the current state of database */
	odb_hash_mask_t hash_mask;	/**< == descr->size - 1 */
	unsigned int sizeof_header;	/**< from base_memory to odb header */
	unsigned int offset_node;	/**< from base_memory to node array */
	void * base_memory;		/**< base memory of the maped memory */
	int fd;				/**< mmaped memory file descriptor */
	char * filename;                /**< full path name of sample file */
	int ref_count;                  /**< reference count */
	struct list_head list;          /**< hash bucket list */
} odb_data_t;

typedef struct {
	odb_data_t * data;
} odb_t;

#ifdef __cplusplus
extern "C" {
#endif

/* db_manage.c */

/** how to open the DB file */
enum odb_rw {
	ODB_RDONLY = 0,	/**< open for read only */
	ODB_RDWR = 1	/**< open for read and/or write */
};

void odb_init(odb_t * odb);

int odb_open(odb_t * odb, char const * filename,
             enum odb_rw rw, size_t sizeof_header);

/** Close the given ODB file */
void odb_close(odb_t * odb);

/** return the number of times this sample file is open */
int odb_open_count(odb_t const * odb);

/** return the start of the mapped data */
void * odb_get_data(odb_t * odb);

/** issue a msync on the used size of the mmaped file */
void odb_sync(odb_t const * odb);

int odb_grow_hashtable(odb_data_t * data);
static __inline void odb_commit_reservation(odb_data_t * data)
{
	++data->descr->current_size;
}

/** "immpossible" node number to indicate an error from odb_hash_add_node() */
#define ODB_NODE_NR_INVALID ((odb_node_nr_t)-1)

/* db_debug.c */
/** check that the hash is well built */
int odb_check_hash(odb_t const * odb);

/* db_stat.c */
typedef struct odb_hash_stat_t odb_hash_stat_t;
odb_hash_stat_t * odb_hash_stat(odb_t const * odb);
void odb_hash_display_stat(odb_hash_stat_t const * stats);
void odb_hash_free_stat(odb_hash_stat_t * stats);

/* db_insert.c */
int odb_update_node(odb_t * odb, odb_key_t key);

int odb_update_node_with_offset(odb_t * odb, 
				odb_key_t key, 
				unsigned long int offset);

int odb_add_node(odb_t * odb, odb_key_t key, odb_value_t value);

/* db_travel.c */
odb_node_t * odb_get_iterator(odb_t const * odb, odb_node_nr_t * nr);

static __inline unsigned int
odb_do_hash(odb_data_t const * data, odb_key_t value)
{
	/* FIXME: better hash for eip value, needs to instrument code
	 * and do a lot of tests ... */
	/* trying to combine high order bits his a no-op: inside a binary image
	 * high order bits don't vary a lot, hash table start with 7 bits mask
	 * so this hash coding use bits 0-7, 8-15. Hash table is stored in
	 * files avoiding to rebuilding them at profiling re-start so
	 * on changing do_hash() change the file format!
	 */
	uint32_t temp = (value >> 32) ^ value;
	return ((temp << 0) ^ (temp >> 8)) & data->hash_mask;
}

#ifdef __cplusplus
}
#endif

#endif /* !ODB_H */
