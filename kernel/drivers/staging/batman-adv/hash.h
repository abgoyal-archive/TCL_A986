

#ifndef _BATMAN_HASH_H
#define _BATMAN_HASH_H
#define HASHIT(name) struct hash_it_t name = { \
		.index = -1, .bucket = NULL, \
		.prev_bucket = NULL, \
		.first_bucket = NULL }


typedef int (*hashdata_compare_cb)(void *, void *);
typedef int (*hashdata_choose_cb)(void *, int);
typedef void (*hashdata_free_cb)(void *);

struct element_t {
	void *data;		/* pointer to the data */
	struct element_t *next;	/* overflow bucket pointer */
};

struct hash_it_t {
	int index;
	struct element_t *bucket;
	struct element_t *prev_bucket;
	struct element_t **first_bucket;
};

struct hashtable_t {
	struct element_t **table;   /* the hashtable itself, with the buckets */
	int elements;		    /* number of elements registered */
	int size;		    /* size of hashtable */
	hashdata_compare_cb compare;/* callback to a compare function.  should
				     * compare 2 element datas for their keys,
				     * return 0 if same and not 0 if not
				     * same */
	hashdata_choose_cb choose;  /* the hashfunction, should return an index
				     * based on the key in the data of the first
				     * argument and the size the second */
};

/* clears the hash */
void hash_init(struct hashtable_t *hash);

/* allocates and clears the hash */
struct hashtable_t *hash_new(int size, hashdata_compare_cb compare,
			     hashdata_choose_cb choose);

void *hash_remove_bucket(struct hashtable_t *hash, struct hash_it_t *hash_it_t);

void hash_delete(struct hashtable_t *hash, hashdata_free_cb free_cb);

/* free only the hashtable and the hash itself. */
void hash_destroy(struct hashtable_t *hash);

/* adds data to the hashtable. returns 0 on success, -1 on error */
int hash_add(struct hashtable_t *hash, void *data);

void *hash_remove(struct hashtable_t *hash, void *data);

void *hash_find(struct hashtable_t *hash, void *keydata);

struct hashtable_t *hash_resize(struct hashtable_t *hash, int size);

struct hash_it_t *hash_iterate(struct hashtable_t *hash,
			       struct hash_it_t *iter_in);

/* print the hash table for debugging */
void hash_debug(struct hashtable_t *hash);
#endif
