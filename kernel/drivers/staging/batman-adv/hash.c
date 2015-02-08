

#include "main.h"
#include "hash.h"

/* clears the hash */
void hash_init(struct hashtable_t *hash)
{
	int i;

	hash->elements = 0;

	for (i = 0 ; i < hash->size; i++)
		hash->table[i] = NULL;
}

void hash_delete(struct hashtable_t *hash, hashdata_free_cb free_cb)
{
	struct element_t *bucket, *last_bucket;
	int i;

	for (i = 0; i < hash->size; i++) {
		bucket = hash->table[i];

		while (bucket != NULL) {
			if (free_cb != NULL)
				free_cb(bucket->data);

			last_bucket = bucket;
			bucket = bucket->next;
			kfree(last_bucket);
		}
	}

	hash_destroy(hash);
}

/* free only the hashtable and the hash itself. */
void hash_destroy(struct hashtable_t *hash)
{
	kfree(hash->table);
	kfree(hash);
}


struct hash_it_t *hash_iterate(struct hashtable_t *hash,
			       struct hash_it_t *iter)
{
	if (!hash)
		return NULL;
	if (!iter)
		return NULL;

	/* sanity checks first (if our bucket got deleted in the last
	 * iteration): */
	if (iter->bucket != NULL) {
		if (iter->first_bucket != NULL) {
			/* we're on the first element and it got removed after
			 * the last iteration. */
			if ((*iter->first_bucket) != iter->bucket) {
				/* there are still other elements in the list */
				if ((*iter->first_bucket) != NULL) {
					iter->prev_bucket = NULL;
					iter->bucket = (*iter->first_bucket);
					iter->first_bucket =
						&hash->table[iter->index];
					return iter;
				} else {
					iter->bucket = NULL;
				}
			}
		} else if (iter->prev_bucket != NULL) {
			/*
			* we're not on the first element, and the bucket got
			* removed after the last iteration.  the last bucket's
			* next pointer is not pointing to our actual bucket
			* anymore.  select the next.
			*/
			if (iter->prev_bucket->next != iter->bucket)
				iter->bucket = iter->prev_bucket;
		}
	}

	/* now as we are sane, select the next one if there is some */
	if (iter->bucket != NULL) {
		if (iter->bucket->next != NULL) {
			iter->prev_bucket = iter->bucket;
			iter->bucket = iter->bucket->next;
			iter->first_bucket = NULL;
			return iter;
		}
	}

	/* if not returned yet, we've reached the last one on the index and have
	 * to search forward */
	iter->index++;
	/* go through the entries of the hash table */
	while (iter->index < hash->size) {
		if ((hash->table[iter->index]) != NULL) {
			iter->prev_bucket = NULL;
			iter->bucket = hash->table[iter->index];
			iter->first_bucket = &hash->table[iter->index];
			return iter;
		} else {
			iter->index++;
		}
	}

	/* nothing to iterate over anymore */
	return NULL;
}

/* allocates and clears the hash */
struct hashtable_t *hash_new(int size, hashdata_compare_cb compare,
			     hashdata_choose_cb choose)
{
	struct hashtable_t *hash;

	hash = kmalloc(sizeof(struct hashtable_t) , GFP_ATOMIC);

	if (hash == NULL)
		return NULL;

	hash->size = size;
	hash->table = kmalloc(sizeof(struct element_t *) * size, GFP_ATOMIC);

	if (hash->table == NULL) {
		kfree(hash);
		return NULL;
	}

	hash_init(hash);

	hash->compare = compare;
	hash->choose = choose;

	return hash;
}

/* adds data to the hashtable. returns 0 on success, -1 on error */
int hash_add(struct hashtable_t *hash, void *data)
{
	int index;
	struct element_t *bucket, *prev_bucket = NULL;

	if (!hash)
		return -1;

	index = hash->choose(data, hash->size);
	bucket = hash->table[index];

	while (bucket != NULL) {
		if (hash->compare(bucket->data, data))
			return -1;

		prev_bucket = bucket;
		bucket = bucket->next;
	}

	/* found the tail of the list, add new element */
	bucket = kmalloc(sizeof(struct element_t), GFP_ATOMIC);

	if (bucket == NULL)
		return -1;

	bucket->data = data;
	bucket->next = NULL;

	/* and link it */
	if (prev_bucket == NULL)
		hash->table[index] = bucket;
	else
		prev_bucket->next = bucket;

	hash->elements++;
	return 0;
}

void *hash_find(struct hashtable_t *hash, void *keydata)
{
	int index;
	struct element_t *bucket;

	if (!hash)
		return NULL;

	index = hash->choose(keydata , hash->size);
	bucket = hash->table[index];

	while (bucket != NULL) {
		if (hash->compare(bucket->data, keydata))
			return bucket->data;

		bucket = bucket->next;
	}

	return NULL;
}

void *hash_remove_bucket(struct hashtable_t *hash, struct hash_it_t *hash_it_t)
{
	void *data_save;

	data_save = hash_it_t->bucket->data;

	if (hash_it_t->prev_bucket != NULL)
		hash_it_t->prev_bucket->next = hash_it_t->bucket->next;
	else if (hash_it_t->first_bucket != NULL)
		(*hash_it_t->first_bucket) = hash_it_t->bucket->next;

	kfree(hash_it_t->bucket);
	hash->elements--;

	return data_save;
}

void *hash_remove(struct hashtable_t *hash, void *data)
{
	struct hash_it_t hash_it_t;

	hash_it_t.index = hash->choose(data, hash->size);
	hash_it_t.bucket = hash->table[hash_it_t.index];
	hash_it_t.prev_bucket = NULL;

	while (hash_it_t.bucket != NULL) {
		if (hash->compare(hash_it_t.bucket->data, data)) {
			hash_it_t.first_bucket =
				(hash_it_t.bucket ==
				 hash->table[hash_it_t.index] ?
				 &hash->table[hash_it_t.index] : NULL);
			return hash_remove_bucket(hash, &hash_it_t);
		}

		hash_it_t.prev_bucket = hash_it_t.bucket;
		hash_it_t.bucket = hash_it_t.bucket->next;
	}

	return NULL;
}

struct hashtable_t *hash_resize(struct hashtable_t *hash, int size)
{
	struct hashtable_t *new_hash;
	struct element_t *bucket;
	int i;

	/* initialize a new hash with the new size */
	new_hash = hash_new(size, hash->compare, hash->choose);

	if (new_hash == NULL)
		return NULL;

	/* copy the elements */
	for (i = 0; i < hash->size; i++) {
		bucket = hash->table[i];

		while (bucket != NULL) {
			hash_add(new_hash, bucket->data);
			bucket = bucket->next;
		}
	}

	/* remove hash and eventual overflow buckets but not the content
	 * itself. */
	hash_delete(hash, NULL);

	return new_hash;
}
