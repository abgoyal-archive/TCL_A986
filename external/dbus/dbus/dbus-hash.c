
/* -*- mode: C; c-file-style: "gnu" -*- */

#include "dbus-hash.h"
#include "dbus-internals.h"
#include "dbus-mempool.h"



#define REBUILD_MULTIPLIER  3

#define RANDOM_INDEX(table, i) \
    (((((long) (i))*1103515245) >> (table)->down_shift) & (table)->mask)

#define DBUS_SMALL_HASH_TABLE 4

typedef struct DBusHashEntry DBusHashEntry;

struct DBusHashEntry
{
  DBusHashEntry *next;    /**< Pointer to next entry in this
                           * hash bucket, or #NULL for end of
                           * chain.
                           */
  void *key;              /**< Hash key */
  void *value;            /**< Hash value */
};

typedef DBusHashEntry* (* DBusFindEntryFunction) (DBusHashTable        *table,
                                                  void                 *key,
                                                  dbus_bool_t           create_if_not_found,
                                                  DBusHashEntry      ***bucket,
                                                  DBusPreallocatedHash *preallocated);

struct DBusHashTable {
  int refcount;                       /**< Reference count */
  
  DBusHashEntry **buckets;            /**< Pointer to bucket array.  Each
                                       * element points to first entry in
                                       * bucket's hash chain, or #NULL.
                                       */
  DBusHashEntry *static_buckets[DBUS_SMALL_HASH_TABLE];
                                       /**< Bucket array used for small tables
                                        * (to avoid mallocs and frees).
                                        */
  int n_buckets;                       /**< Total number of buckets allocated
                                        * at **buckets.
                                        */
  int n_entries;                       /**< Total number of entries present
                                        * in table.
                                        */
  int hi_rebuild_size;                 /**< Enlarge table when n_entries gets
                                        * to be this large.
                                        */
  int lo_rebuild_size;                 /**< Shrink table when n_entries gets
                                        * below this.
                                        */
  int down_shift;                      /**< Shift count used in hashing
                                        * function.  Designed to use high-
                                        * order bits of randomized keys.
                                        */
  int mask;                            /**< Mask value used in hashing
                                        * function.
                                        */
  DBusHashType key_type;               /**< Type of keys used in this table */


  DBusFindEntryFunction find_function; /**< Function for finding entries */

  DBusFreeFunction free_key_function;   /**< Function to free keys */
  DBusFreeFunction free_value_function; /**< Function to free values */

  DBusMemPool *entry_pool;              /**< Memory pool for hash entries */
};

typedef struct
{
  DBusHashTable *table;     /**< Pointer to table containing entry. */
  DBusHashEntry **bucket;   /**< Pointer to bucket that points to
                             * first entry in this entry's chain:
                             * used for deleting the entry.
                             */
  DBusHashEntry *entry;      /**< Current hash entry */
  DBusHashEntry *next_entry; /**< Next entry to be iterated onto in current bucket */
  int next_bucket;           /**< index of next bucket */
  int n_entries_on_init;     /**< used to detect table resize since initialization */
} DBusRealHashIter;

static DBusHashEntry* find_direct_function      (DBusHashTable          *table,
                                                 void                   *key,
                                                 dbus_bool_t             create_if_not_found,
                                                 DBusHashEntry        ***bucket,
                                                 DBusPreallocatedHash   *preallocated);
static DBusHashEntry* find_string_function      (DBusHashTable          *table,
                                                 void                   *key,
                                                 dbus_bool_t             create_if_not_found,
                                                 DBusHashEntry        ***bucket,
                                                 DBusPreallocatedHash   *preallocated);
#ifdef DBUS_BUILD_TESTS
static DBusHashEntry* find_two_strings_function (DBusHashTable          *table,
                                                 void                   *key,
                                                 dbus_bool_t             create_if_not_found,
                                                 DBusHashEntry        ***bucket,
                                                 DBusPreallocatedHash   *preallocated);
#endif
static unsigned int   string_hash               (const char             *str);
#ifdef DBUS_BUILD_TESTS
static unsigned int   two_strings_hash          (const char             *str);
#endif
static void           rebuild_table             (DBusHashTable          *table);
static DBusHashEntry* alloc_entry               (DBusHashTable          *table);
static void           remove_entry              (DBusHashTable          *table,
                                                 DBusHashEntry         **bucket,
                                                 DBusHashEntry          *entry);
static void           free_entry                (DBusHashTable          *table,
                                                 DBusHashEntry          *entry);
static void           free_entry_data           (DBusHashTable          *table,
                                                 DBusHashEntry          *entry);


/** @} */





DBusHashTable*
_dbus_hash_table_new (DBusHashType     type,
                      DBusFreeFunction key_free_function,
                      DBusFreeFunction value_free_function)
{
  DBusHashTable *table;
  DBusMemPool *entry_pool;
  
  table = dbus_new0 (DBusHashTable, 1);
  if (table == NULL)
    return NULL;

  entry_pool = _dbus_mem_pool_new (sizeof (DBusHashEntry), TRUE);
  if (entry_pool == NULL)
    {
      dbus_free (table);
      return NULL;
    }
  
  table->refcount = 1;
  table->entry_pool = entry_pool;
  
  _dbus_assert (DBUS_SMALL_HASH_TABLE == _DBUS_N_ELEMENTS (table->static_buckets));
  
  table->buckets = table->static_buckets;  
  table->n_buckets = DBUS_SMALL_HASH_TABLE;
  table->n_entries = 0;
  table->hi_rebuild_size = DBUS_SMALL_HASH_TABLE * REBUILD_MULTIPLIER;
  table->lo_rebuild_size = 0;
  table->down_shift = 28;
  table->mask = 3;
  table->key_type = type;

  _dbus_assert (table->mask < table->n_buckets);
  
  switch (table->key_type)
    {
    case DBUS_HASH_INT:
    case DBUS_HASH_POINTER:
    case DBUS_HASH_ULONG:
      table->find_function = find_direct_function;
      break;
    case DBUS_HASH_STRING:
      table->find_function = find_string_function;
      break;
    case DBUS_HASH_TWO_STRINGS:
#ifdef DBUS_BUILD_TESTS
      table->find_function = find_two_strings_function;
#endif
      break;
    default:
      _dbus_assert_not_reached ("Unknown hash table type");
      break;
    }

  table->free_key_function = key_free_function;
  table->free_value_function = value_free_function;

  return table;
}


DBusHashTable *
_dbus_hash_table_ref (DBusHashTable *table)
{
  table->refcount += 1;
  
  return table;
}

void
_dbus_hash_table_unref (DBusHashTable *table)
{
  table->refcount -= 1;

  if (table->refcount == 0)
    {
#if 0
      DBusHashEntry *entry;
      DBusHashEntry *next;
      int i;

      /* Free the entries in the table. */
      for (i = 0; i < table->n_buckets; i++)
        {
          entry = table->buckets[i];
          while (entry != NULL)
            {
              next = entry->next;

              free_entry (table, entry);
              
              entry = next;
            }
        }
#else
      DBusHashEntry *entry;
      int i;

      /* Free the entries in the table. */
      for (i = 0; i < table->n_buckets; i++)
        {
          entry = table->buckets[i];
          while (entry != NULL)
            {
              free_entry_data (table, entry);
              
              entry = entry->next;
            }
        }
      /* We can do this very quickly with memory pools ;-) */
      _dbus_mem_pool_free (table->entry_pool);
#endif
      
      /* Free the bucket array, if it was dynamically allocated. */
      if (table->buckets != table->static_buckets)
        dbus_free (table->buckets);

      dbus_free (table);
    }
}

void
_dbus_hash_table_remove_all (DBusHashTable *table)
{
  DBusHashIter iter;
  _dbus_hash_iter_init (table, &iter);
  while (_dbus_hash_iter_next (&iter))
    {
      _dbus_hash_iter_remove_entry(&iter);
    }
}

static DBusHashEntry*
alloc_entry (DBusHashTable *table)
{
  DBusHashEntry *entry;

  entry = _dbus_mem_pool_alloc (table->entry_pool);
  
  return entry;
}

static void
free_entry_data (DBusHashTable  *table,
		 DBusHashEntry  *entry)
{
  if (table->free_key_function)
    (* table->free_key_function) (entry->key);
  if (table->free_value_function)
    (* table->free_value_function) (entry->value);
}

static void
free_entry (DBusHashTable  *table,
            DBusHashEntry  *entry)
{
  free_entry_data (table, entry);
  _dbus_mem_pool_dealloc (table->entry_pool, entry);
}

static void
remove_entry (DBusHashTable  *table,
              DBusHashEntry **bucket,
              DBusHashEntry  *entry)
{
  _dbus_assert (table != NULL);
  _dbus_assert (bucket != NULL);
  _dbus_assert (*bucket != NULL);  
  _dbus_assert (entry != NULL);
  
  if (*bucket == entry)
    *bucket = entry->next;
  else
    {
      DBusHashEntry *prev;
      prev = *bucket;

      while (prev->next != entry)
        prev = prev->next;      
      
      _dbus_assert (prev != NULL);

      prev->next = entry->next;
    }
  
  table->n_entries -= 1;
  free_entry (table, entry);
}

void
_dbus_hash_iter_init (DBusHashTable *table,
                      DBusHashIter  *iter)
{
  DBusRealHashIter *real;
  
  _dbus_assert (sizeof (DBusHashIter) == sizeof (DBusRealHashIter));
  
  real = (DBusRealHashIter*) iter;

  real->table = table;
  real->bucket = NULL;
  real->entry = NULL;
  real->next_entry = NULL;
  real->next_bucket = 0;
  real->n_entries_on_init = table->n_entries;
}

dbus_bool_t
_dbus_hash_iter_next (DBusHashIter  *iter)
{
  DBusRealHashIter *real;
  
  _dbus_assert (sizeof (DBusHashIter) == sizeof (DBusRealHashIter));
  
  real = (DBusRealHashIter*) iter;

  /* if this assertion failed someone probably added hash entries
   * during iteration, which is bad.
   */
  _dbus_assert (real->n_entries_on_init >= real->table->n_entries);
  
  /* Remember that real->entry may have been deleted */
  
  while (real->next_entry == NULL)
    {
      if (real->next_bucket >= real->table->n_buckets)
        {
          /* invalidate iter and return false */
          real->entry = NULL;
          real->table = NULL;
          real->bucket = NULL;
          return FALSE;
        }

      real->bucket = &(real->table->buckets[real->next_bucket]);
      real->next_entry = *(real->bucket);
      real->next_bucket += 1;
    }

  _dbus_assert (real->next_entry != NULL);
  _dbus_assert (real->bucket != NULL);
  
  real->entry = real->next_entry;
  real->next_entry = real->entry->next;
  
  return TRUE;
}

void
_dbus_hash_iter_remove_entry (DBusHashIter *iter)
{
  DBusRealHashIter *real;

  real = (DBusRealHashIter*) iter;

  _dbus_assert (real->table != NULL);
  _dbus_assert (real->entry != NULL);
  _dbus_assert (real->bucket != NULL);
  
  remove_entry (real->table, real->bucket, real->entry);

  real->entry = NULL; /* make it crash if you try to use this entry */
}

void*
_dbus_hash_iter_get_value (DBusHashIter *iter)
{
  DBusRealHashIter *real;

  real = (DBusRealHashIter*) iter;

  _dbus_assert (real->table != NULL);
  _dbus_assert (real->entry != NULL);

  return real->entry->value;
}

void
_dbus_hash_iter_set_value (DBusHashIter *iter,
                           void         *value)
{
  DBusRealHashIter *real;

  real = (DBusRealHashIter*) iter;

  _dbus_assert (real->table != NULL);
  _dbus_assert (real->entry != NULL);

  if (real->table->free_value_function && value != real->entry->value)    
    (* real->table->free_value_function) (real->entry->value);
  
  real->entry->value = value;
}

int
_dbus_hash_iter_get_int_key (DBusHashIter *iter)
{
  DBusRealHashIter *real;

  real = (DBusRealHashIter*) iter;

  _dbus_assert (real->table != NULL);
  _dbus_assert (real->entry != NULL);

  return _DBUS_POINTER_TO_INT (real->entry->key);
}

unsigned long
_dbus_hash_iter_get_ulong_key (DBusHashIter *iter)
{
  DBusRealHashIter *real;

  real = (DBusRealHashIter*) iter;

  _dbus_assert (real->table != NULL);
  _dbus_assert (real->entry != NULL);

  return (unsigned long) real->entry->key;
}

const char*
_dbus_hash_iter_get_string_key (DBusHashIter *iter)
{
  DBusRealHashIter *real;

  real = (DBusRealHashIter*) iter;

  _dbus_assert (real->table != NULL);
  _dbus_assert (real->entry != NULL);

  return real->entry->key;
}

#ifdef DBUS_BUILD_TESTS
const char*
_dbus_hash_iter_get_two_strings_key (DBusHashIter *iter)
{
  DBusRealHashIter *real;

  real = (DBusRealHashIter*) iter;

  _dbus_assert (real->table != NULL);
  _dbus_assert (real->entry != NULL);

  return real->entry->key;
}
#endif /* DBUS_BUILD_TESTS */

dbus_bool_t
_dbus_hash_iter_lookup (DBusHashTable *table,
                        void          *key,
                        dbus_bool_t    create_if_not_found,
                        DBusHashIter  *iter)
{
  DBusRealHashIter *real;
  DBusHashEntry *entry;
  DBusHashEntry **bucket;
  
  _dbus_assert (sizeof (DBusHashIter) == sizeof (DBusRealHashIter));
  
  real = (DBusRealHashIter*) iter;

  entry = (* table->find_function) (table, key, create_if_not_found, &bucket, NULL);

  if (entry == NULL)
    return FALSE;
  
  real->table = table;
  real->bucket = bucket;
  real->entry = entry;
  real->next_entry = entry->next;
  real->next_bucket = (bucket - table->buckets) + 1;
  real->n_entries_on_init = table->n_entries; 

  _dbus_assert (&(table->buckets[real->next_bucket-1]) == real->bucket);
  
  return TRUE;
}

static void
add_allocated_entry (DBusHashTable   *table,
                     DBusHashEntry   *entry,
                     unsigned int     idx,
                     void            *key,
                     DBusHashEntry ***bucket)
{
  DBusHashEntry **b;  
  
  entry->key = key;
  
  b = &(table->buckets[idx]);
  entry->next = *b;
  *b = entry;

  if (bucket)
    *bucket = b;
  
  table->n_entries += 1;

  /* note we ONLY rebuild when ADDING - because you can iterate over a
   * table and remove entries safely.
   */
  if (table->n_entries >= table->hi_rebuild_size ||
      table->n_entries < table->lo_rebuild_size)
    rebuild_table (table);
}

static DBusHashEntry*
add_entry (DBusHashTable        *table, 
           unsigned int          idx,
           void                 *key,
           DBusHashEntry      ***bucket,
           DBusPreallocatedHash *preallocated)
{
  DBusHashEntry  *entry;

  if (preallocated == NULL)
    {
      entry = alloc_entry (table);
      if (entry == NULL)
        {
          if (bucket)
            *bucket = NULL;
          return NULL;
        }
    }
  else
    {
      entry = (DBusHashEntry*) preallocated;
    }

  add_allocated_entry (table, entry, idx, key, bucket);

  return entry;
}

static unsigned int
string_hash (const char *str)
{
  const char *p = str;
  unsigned int h = *p;

  if (h)
    for (p += 1; *p != '\0'; p++)
      h = (h << 5) - h + *p;

  return h;
}

#ifdef DBUS_BUILD_TESTS
static unsigned int
two_strings_hash (const char *str)
{
  const char *p = str;
  unsigned int h = *p;

  if (h)
    for (p += 1; *p != '\0'; p++)
      h = (h << 5) - h + *p;

  for (p += 1; *p != '\0'; p++)
    h = (h << 5) - h + *p;
  
  return h;
}
#endif /* DBUS_BUILD_TESTS */

/** Key comparison function */
typedef int (* KeyCompareFunc) (const void *key_a, const void *key_b);

static DBusHashEntry*
find_generic_function (DBusHashTable        *table,
                       void                 *key,
                       unsigned int          idx,
                       KeyCompareFunc        compare_func,
                       dbus_bool_t           create_if_not_found,
                       DBusHashEntry      ***bucket,
                       DBusPreallocatedHash *preallocated)
{
  DBusHashEntry *entry;

  if (bucket)
    *bucket = NULL;

  /* Search all of the entries in this bucket. */
  entry = table->buckets[idx];
  while (entry != NULL)
    {
      if ((compare_func == NULL && key == entry->key) ||
          (compare_func != NULL && (* compare_func) (key, entry->key) == 0))
        {
          if (bucket)
            *bucket = &(table->buckets[idx]);

          if (preallocated)
            _dbus_hash_table_free_preallocated_entry (table, preallocated);
          
          return entry;
        }
      
      entry = entry->next;
    }

  if (create_if_not_found)
    entry = add_entry (table, idx, key, bucket, preallocated);
  else if (preallocated)
    _dbus_hash_table_free_preallocated_entry (table, preallocated);
  
  return entry;
}

static DBusHashEntry*
find_string_function (DBusHashTable        *table,
                      void                 *key,
                      dbus_bool_t           create_if_not_found,
                      DBusHashEntry      ***bucket,
                      DBusPreallocatedHash *preallocated)
{
  unsigned int idx;
  
  idx = string_hash (key) & table->mask;

  return find_generic_function (table, key, idx,
                                (KeyCompareFunc) strcmp, create_if_not_found, bucket,
                                preallocated);
}

#ifdef DBUS_BUILD_TESTS
static int
two_strings_cmp (const char *a,
                 const char *b)
{
  size_t len_a;
  size_t len_b;
  int res;
  
  res = strcmp (a, b);
  if (res != 0)
    return res;

  len_a = strlen (a);
  len_b = strlen (b);

  return strcmp (a + len_a + 1, b + len_b + 1);
}
#endif

#ifdef DBUS_BUILD_TESTS
static DBusHashEntry*
find_two_strings_function (DBusHashTable        *table,
                           void                 *key,
                           dbus_bool_t           create_if_not_found,
                           DBusHashEntry      ***bucket,
                           DBusPreallocatedHash *preallocated)
{
  unsigned int idx;
  
  idx = two_strings_hash (key) & table->mask;

  return find_generic_function (table, key, idx,
                                (KeyCompareFunc) two_strings_cmp, create_if_not_found, bucket,
                                preallocated);
}
#endif /* DBUS_BUILD_TESTS */

static DBusHashEntry*
find_direct_function (DBusHashTable        *table,
                      void                 *key,
                      dbus_bool_t           create_if_not_found,
                      DBusHashEntry      ***bucket,
                      DBusPreallocatedHash *preallocated)
{
  unsigned int idx;
  
  idx = RANDOM_INDEX (table, key) & table->mask;


  return find_generic_function (table, key, idx,
                                NULL, create_if_not_found, bucket,
                                preallocated);
}

static void
rebuild_table (DBusHashTable *table)
{
  int old_size;
  int new_buckets;
  DBusHashEntry **old_buckets;
  DBusHashEntry **old_chain;
  DBusHashEntry *entry;
  dbus_bool_t growing;
  
  /*
   * Allocate and initialize the new bucket array, and set up
   * hashing constants for new array size.
   */

  growing = table->n_entries >= table->hi_rebuild_size;
  
  old_size = table->n_buckets;
  old_buckets = table->buckets;

  if (growing)
    {
      /* overflow paranoia */
      if (table->n_buckets < _DBUS_INT_MAX / 4 &&
          table->down_shift >= 0)
        new_buckets = table->n_buckets * 4;
      else
        return; /* can't grow anymore */
    }
  else
    {
      new_buckets = table->n_buckets / 4;
      if (new_buckets < DBUS_SMALL_HASH_TABLE)
        return; /* don't bother shrinking this far */
    }

  table->buckets = dbus_new0 (DBusHashEntry*, new_buckets);
  if (table->buckets == NULL)
    {
      /* out of memory, yay - just don't reallocate, the table will
       * still work, albeit more slowly.
       */
      table->buckets = old_buckets;
      return;
    }

  table->n_buckets = new_buckets;
  
  if (growing)
    {
      table->lo_rebuild_size = table->hi_rebuild_size;
      table->hi_rebuild_size *= 4;
      
      table->down_shift -= 2;               /* keep 2 more high bits */
      table->mask = (table->mask << 2) + 3; /* keep 2 more high bits */
    }
  else
    {
      table->hi_rebuild_size = table->lo_rebuild_size;
      table->lo_rebuild_size /= 4;

      table->down_shift += 2;         /* keep 2 fewer high bits */
      table->mask = table->mask >> 2; /* keep 2 fewer high bits */
    }

#if 0
  printf ("%s table to lo = %d hi = %d downshift = %d mask = 0x%x\n",
          growing ? "GROW" : "SHRINK",
          table->lo_rebuild_size,
          table->hi_rebuild_size,
          table->down_shift,
          table->mask);
#endif
  
  _dbus_assert (table->lo_rebuild_size >= 0);
  _dbus_assert (table->hi_rebuild_size > table->lo_rebuild_size);
  _dbus_assert (table->mask != 0);
  /* the mask is essentially the max index */
  _dbus_assert (table->mask < table->n_buckets);
  
  /*
   * Rehash all of the existing entries into the new bucket array.
   */

  for (old_chain = old_buckets; old_size > 0; old_size--, old_chain++)
    {
      for (entry = *old_chain; entry != NULL; entry = *old_chain)
        {
          unsigned int idx;
          DBusHashEntry **bucket;
          
          *old_chain = entry->next;
          switch (table->key_type)
            {
            case DBUS_HASH_STRING:
              idx = string_hash (entry->key) & table->mask;
              break;
            case DBUS_HASH_TWO_STRINGS:
#ifdef DBUS_BUILD_TESTS
              idx = two_strings_hash (entry->key) & table->mask;
#else
              idx = 0;
              _dbus_assert_not_reached ("two-strings is not enabled");
#endif
              break;
            case DBUS_HASH_INT:
            case DBUS_HASH_ULONG:
            case DBUS_HASH_POINTER:
              idx = RANDOM_INDEX (table, entry->key);
              break;
            default:
              idx = 0;
              _dbus_assert_not_reached ("Unknown hash table type");
              break;
            }
          
          bucket = &(table->buckets[idx]);
          entry->next = *bucket;
          *bucket = entry;
        }
    }
  
  /* Free the old bucket array, if it was dynamically allocated. */

  if (old_buckets != table->static_buckets)
    dbus_free (old_buckets);
}

void*
_dbus_hash_table_lookup_string (DBusHashTable *table,
                                const char    *key)
{
  DBusHashEntry *entry;

  _dbus_assert (table->key_type == DBUS_HASH_STRING);
  
  entry = (* table->find_function) (table, (char*) key, FALSE, NULL, NULL);

  if (entry)
    return entry->value;
  else
    return NULL;
}

#ifdef DBUS_BUILD_TESTS
void*
_dbus_hash_table_lookup_two_strings (DBusHashTable *table,
                                     const char    *key)
{
  DBusHashEntry *entry;

  _dbus_assert (table->key_type == DBUS_HASH_TWO_STRINGS);
  
  entry = (* table->find_function) (table, (char*) key, FALSE, NULL, NULL);

  if (entry)
    return entry->value;
  else
    return NULL;
}
#endif /* DBUS_BUILD_TESTS */

void*
_dbus_hash_table_lookup_int (DBusHashTable *table,
                             int            key)
{
  DBusHashEntry *entry;

  _dbus_assert (table->key_type == DBUS_HASH_INT);
  
  entry = (* table->find_function) (table, _DBUS_INT_TO_POINTER (key), FALSE, NULL, NULL);

  if (entry)
    return entry->value;
  else
    return NULL;
}

#ifdef DBUS_BUILD_TESTS
/* disabled since it's only used for testing */
void*
_dbus_hash_table_lookup_pointer (DBusHashTable *table,
                                 void          *key)
{
  DBusHashEntry *entry;

  _dbus_assert (table->key_type == DBUS_HASH_POINTER);
  
  entry = (* table->find_function) (table, key, FALSE, NULL, NULL);

  if (entry)
    return entry->value;
  else
    return NULL;
}
#endif /* DBUS_BUILD_TESTS */

void*
_dbus_hash_table_lookup_ulong (DBusHashTable *table,
                               unsigned long  key)
{
  DBusHashEntry *entry;

  _dbus_assert (table->key_type == DBUS_HASH_ULONG);
  
  entry = (* table->find_function) (table, (void*) key, FALSE, NULL, NULL);

  if (entry)
    return entry->value;
  else
    return NULL;
}

dbus_bool_t
_dbus_hash_table_remove_string (DBusHashTable *table,
                                const char    *key)
{
  DBusHashEntry *entry;
  DBusHashEntry **bucket;
  
  _dbus_assert (table->key_type == DBUS_HASH_STRING);
  
  entry = (* table->find_function) (table, (char*) key, FALSE, &bucket, NULL);

  if (entry)
    {
      remove_entry (table, bucket, entry);
      return TRUE;
    }
  else
    return FALSE;
}

#ifdef DBUS_BUILD_TESTS
dbus_bool_t
_dbus_hash_table_remove_two_strings (DBusHashTable *table,
                                     const char    *key)
{
  DBusHashEntry *entry;
  DBusHashEntry **bucket;
  
  _dbus_assert (table->key_type == DBUS_HASH_TWO_STRINGS);
  
  entry = (* table->find_function) (table, (char*) key, FALSE, &bucket, NULL);

  if (entry)
    {
      remove_entry (table, bucket, entry);
      return TRUE;
    }
  else
    return FALSE;
}
#endif /* DBUS_BUILD_TESTS */

dbus_bool_t
_dbus_hash_table_remove_int (DBusHashTable *table,
                             int            key)
{
  DBusHashEntry *entry;
  DBusHashEntry **bucket;
  
  _dbus_assert (table->key_type == DBUS_HASH_INT);
  
  entry = (* table->find_function) (table, _DBUS_INT_TO_POINTER (key), FALSE, &bucket, NULL);
  
  if (entry)
    {
      remove_entry (table, bucket, entry);
      return TRUE;
    }
  else
    return FALSE;
}

#ifdef DBUS_BUILD_TESTS
/* disabled since it's only used for testing */
dbus_bool_t
_dbus_hash_table_remove_pointer (DBusHashTable *table,
                                 void          *key)
{
  DBusHashEntry *entry;
  DBusHashEntry **bucket;
  
  _dbus_assert (table->key_type == DBUS_HASH_POINTER);
  
  entry = (* table->find_function) (table, key, FALSE, &bucket, NULL);
  
  if (entry)
    {
      remove_entry (table, bucket, entry);
      return TRUE;
    }
  else
    return FALSE;
}
#endif /* DBUS_BUILD_TESTS */

dbus_bool_t
_dbus_hash_table_remove_ulong (DBusHashTable *table,
                               unsigned long  key)
{
  DBusHashEntry *entry;
  DBusHashEntry **bucket;
  
  _dbus_assert (table->key_type == DBUS_HASH_ULONG);
  
  entry = (* table->find_function) (table, (void*) key, FALSE, &bucket, NULL);
  
  if (entry)
    {
      remove_entry (table, bucket, entry);
      return TRUE;
    }
  else
    return FALSE;
}

dbus_bool_t
_dbus_hash_table_insert_string (DBusHashTable *table,
                                char          *key,
                                void          *value)
{
  DBusPreallocatedHash *preallocated;

  _dbus_assert (table->key_type == DBUS_HASH_STRING);

  preallocated = _dbus_hash_table_preallocate_entry (table);
  if (preallocated == NULL)
    return FALSE;

  _dbus_hash_table_insert_string_preallocated (table, preallocated,
                                               key, value);
  
  return TRUE;
}

#ifdef DBUS_BUILD_TESTS
dbus_bool_t
_dbus_hash_table_insert_two_strings (DBusHashTable *table,
                                     char          *key,
                                     void          *value)
{
  DBusHashEntry *entry;
  
  _dbus_assert (table->key_type == DBUS_HASH_TWO_STRINGS);
  
  entry = (* table->find_function) (table, key, TRUE, NULL, NULL);

  if (entry == NULL)
    return FALSE; /* no memory */

  if (table->free_key_function && entry->key != key)
    (* table->free_key_function) (entry->key);
  
  if (table->free_value_function && entry->value != value)
    (* table->free_value_function) (entry->value);
  
  entry->key = key;
  entry->value = value;

  return TRUE;
}
#endif /* DBUS_BUILD_TESTS */

dbus_bool_t
_dbus_hash_table_insert_int (DBusHashTable *table,
                             int            key,
                             void          *value)
{
  DBusHashEntry *entry;

  _dbus_assert (table->key_type == DBUS_HASH_INT);
  
  entry = (* table->find_function) (table, _DBUS_INT_TO_POINTER (key), TRUE, NULL, NULL);

  if (entry == NULL)
    return FALSE; /* no memory */

  if (table->free_key_function && entry->key != _DBUS_INT_TO_POINTER (key))
    (* table->free_key_function) (entry->key);
  
  if (table->free_value_function && entry->value != value)
    (* table->free_value_function) (entry->value);
  
  entry->key = _DBUS_INT_TO_POINTER (key);
  entry->value = value;

  return TRUE;
}

#ifdef DBUS_BUILD_TESTS
/* disabled since it's only used for testing */
dbus_bool_t
_dbus_hash_table_insert_pointer (DBusHashTable *table,
                                 void          *key,
                                 void          *value)
{
  DBusHashEntry *entry;

  _dbus_assert (table->key_type == DBUS_HASH_POINTER);
  
  entry = (* table->find_function) (table, key, TRUE, NULL, NULL);

  if (entry == NULL)
    return FALSE; /* no memory */

  if (table->free_key_function && entry->key != key)
    (* table->free_key_function) (entry->key);
  
  if (table->free_value_function && entry->value != value)
    (* table->free_value_function) (entry->value);
  
  entry->key = key;
  entry->value = value;

  return TRUE;
}
#endif /* DBUS_BUILD_TESTS */

dbus_bool_t
_dbus_hash_table_insert_ulong (DBusHashTable *table,
                               unsigned long  key,
                               void          *value)
{
  DBusHashEntry *entry;

  _dbus_assert (table->key_type == DBUS_HASH_ULONG);
  
  entry = (* table->find_function) (table, (void*) key, TRUE, NULL, NULL);

  if (entry == NULL)
    return FALSE; /* no memory */

  if (table->free_key_function && entry->key != (void*) key)
    (* table->free_key_function) (entry->key);
  
  if (table->free_value_function && entry->value != value)
    (* table->free_value_function) (entry->value);
  
  entry->key = (void*) key;
  entry->value = value;

  return TRUE;
}

DBusPreallocatedHash*
_dbus_hash_table_preallocate_entry (DBusHashTable *table)
{
  DBusHashEntry *entry;
  
  entry = alloc_entry (table);

  return (DBusPreallocatedHash*) entry;
}

void
_dbus_hash_table_free_preallocated_entry (DBusHashTable        *table,
                                          DBusPreallocatedHash *preallocated)
{
  DBusHashEntry *entry;

  _dbus_assert (preallocated != NULL);
  
  entry = (DBusHashEntry*) preallocated;
  
  /* Don't use free_entry(), since this entry has no key/data */
  _dbus_mem_pool_dealloc (table->entry_pool, entry);
}

void
_dbus_hash_table_insert_string_preallocated (DBusHashTable        *table,
                                             DBusPreallocatedHash *preallocated,
                                             char                 *key,
                                             void                 *value)
{
  DBusHashEntry *entry;

  _dbus_assert (table->key_type == DBUS_HASH_STRING);
  _dbus_assert (preallocated != NULL);
  
  entry = (* table->find_function) (table, key, TRUE, NULL, preallocated);

  _dbus_assert (entry != NULL);
  
  if (table->free_key_function && entry->key != key)
    (* table->free_key_function) (entry->key);

  if (table->free_value_function && entry->value != value)
    (* table->free_value_function) (entry->value);
      
  entry->key = key;
  entry->value = value;
}

int
_dbus_hash_table_get_n_entries (DBusHashTable *table)
{
  return table->n_entries;
}

/** @} */

#ifdef DBUS_BUILD_TESTS
#include "dbus-test.h"
#include <stdio.h>

static int
count_entries (DBusHashTable *table)
{
  DBusHashIter iter;
  int count;

  count = 0;
  _dbus_hash_iter_init (table, &iter);
  while (_dbus_hash_iter_next (&iter))
    ++count;

  _dbus_assert (count == _dbus_hash_table_get_n_entries (table));
  
  return count;
}

/* Copy the foo\0bar\0 double string thing */
static char*
_dbus_strdup2 (const char *str)
{
  size_t len;
  char *copy;
  
  if (str == NULL)
    return NULL;
  
  len = strlen (str);
  len += strlen ((str + len + 1));

  copy = dbus_malloc (len + 2);
  if (copy == NULL)
    return NULL;

  memcpy (copy, str, len + 2);
  
  return copy;
}

dbus_bool_t
_dbus_hash_test (void)
{
  int i;
  DBusHashTable *table1;
  DBusHashTable *table2;
  DBusHashTable *table3;
  DBusHashTable *table4;
  DBusHashIter iter;
#define N_HASH_KEYS 5000
  char **keys;
  dbus_bool_t ret = FALSE;

  keys = dbus_new (char *, N_HASH_KEYS);
  if (keys == NULL)
    _dbus_assert_not_reached ("no memory");

  for (i = 0; i < N_HASH_KEYS; i++)
    {
      keys[i] = dbus_malloc (128);

      if (keys[i] == NULL)
	_dbus_assert_not_reached ("no memory");
    }

  printf ("Computing test hash keys...\n");
  i = 0;
  while (i < N_HASH_KEYS)
    {
      int len;

      /* all the hash keys are TWO_STRINGS, but
       * then we can also use those as regular strings.
       */
      
      len = sprintf (keys[i], "Hash key %d", i);
      sprintf (keys[i] + len + 1, "Two string %d", i);
      _dbus_assert (*(keys[i] + len) == '\0');
      _dbus_assert (*(keys[i] + len + 1) != '\0');
      ++i;
    }
  printf ("... done.\n");
  
  table1 = _dbus_hash_table_new (DBUS_HASH_STRING,
                                 dbus_free, dbus_free);
  if (table1 == NULL)
    goto out;

  table2 = _dbus_hash_table_new (DBUS_HASH_INT,
                                 NULL, dbus_free);
  if (table2 == NULL)
    goto out;

  table3 = _dbus_hash_table_new (DBUS_HASH_ULONG,
                                 NULL, dbus_free);
  if (table3 == NULL)
    goto out;

  table4 = _dbus_hash_table_new (DBUS_HASH_TWO_STRINGS,
                                 dbus_free, dbus_free);
  if (table4 == NULL)
    goto out;

  
  /* Insert and remove a bunch of stuff, counting the table in between
   * to be sure it's not broken and that iteration works
   */
  i = 0;
  while (i < 3000)
    {
      void *value;
      char *key;

      key = _dbus_strdup (keys[i]);
      if (key == NULL)
        goto out;
      value = _dbus_strdup ("Value!");
      if (value == NULL)
        goto out;
      
      if (!_dbus_hash_table_insert_string (table1,
                                           key, value))
        goto out;

      value = _dbus_strdup (keys[i]);
      if (value == NULL)
        goto out;
      
      if (!_dbus_hash_table_insert_int (table2,
                                        i, value))
        goto out;

      value = _dbus_strdup (keys[i]);
      if (value == NULL)
        goto out;
      
      if (!_dbus_hash_table_insert_ulong (table3,
                                          i, value))
        goto out;

      key = _dbus_strdup2 (keys[i]);
      if (key == NULL)
        goto out;
      value = _dbus_strdup ("Value!");
      if (value == NULL)
        goto out;
      
      if (!_dbus_hash_table_insert_two_strings (table4,
                                                key, value))
        goto out;
      
      _dbus_assert (count_entries (table1) == i + 1);
      _dbus_assert (count_entries (table2) == i + 1);
      _dbus_assert (count_entries (table3) == i + 1);
      _dbus_assert (count_entries (table4) == i + 1);

      value = _dbus_hash_table_lookup_string (table1, keys[i]);
      _dbus_assert (value != NULL);
      _dbus_assert (strcmp (value, "Value!") == 0);

      value = _dbus_hash_table_lookup_int (table2, i);
      _dbus_assert (value != NULL);
      _dbus_assert (strcmp (value, keys[i]) == 0);

      value = _dbus_hash_table_lookup_ulong (table3, i);
      _dbus_assert (value != NULL);
      _dbus_assert (strcmp (value, keys[i]) == 0);

      value = _dbus_hash_table_lookup_two_strings (table4, keys[i]);
      _dbus_assert (value != NULL);
      _dbus_assert (strcmp (value, "Value!") == 0);
      
      ++i;
    }

  --i;
  while (i >= 0)
    {
      _dbus_hash_table_remove_string (table1,
                                      keys[i]);

      _dbus_hash_table_remove_int (table2, i);

      _dbus_hash_table_remove_ulong (table3, i); 

      _dbus_hash_table_remove_two_strings (table4,
                                           keys[i]);
      
      _dbus_assert (count_entries (table1) == i);
      _dbus_assert (count_entries (table2) == i);
      _dbus_assert (count_entries (table3) == i);
      _dbus_assert (count_entries (table4) == i);

      --i;
    }

  _dbus_hash_table_ref (table1);
  _dbus_hash_table_ref (table2);
  _dbus_hash_table_ref (table3);
  _dbus_hash_table_ref (table4);
  _dbus_hash_table_unref (table1);
  _dbus_hash_table_unref (table2);
  _dbus_hash_table_unref (table3);
  _dbus_hash_table_unref (table4);
  _dbus_hash_table_unref (table1);
  _dbus_hash_table_unref (table2);
  _dbus_hash_table_unref (table3);
  _dbus_hash_table_unref (table4);
  table3 = NULL;

  /* Insert a bunch of stuff then check
   * that iteration works correctly (finds the right
   * values, iter_set_value works, etc.)
   */
  table1 = _dbus_hash_table_new (DBUS_HASH_STRING,
                                 dbus_free, dbus_free);
  if (table1 == NULL)
    goto out;
  
  table2 = _dbus_hash_table_new (DBUS_HASH_INT,
                                 NULL, dbus_free);
  if (table2 == NULL)
    goto out;
  
  i = 0;
  while (i < 5000)
    {
      char *key;
      void *value;      
      
      key = _dbus_strdup (keys[i]);
      if (key == NULL)
        goto out;
      value = _dbus_strdup ("Value!");
      if (value == NULL)
        goto out;
      
      if (!_dbus_hash_table_insert_string (table1,
                                           key, value))
        goto out;

      value = _dbus_strdup (keys[i]);
      if (value == NULL)
        goto out;
      
      if (!_dbus_hash_table_insert_int (table2,
                                        i, value))
        goto out;
      
      _dbus_assert (count_entries (table1) == i + 1);
      _dbus_assert (count_entries (table2) == i + 1);
      
      ++i;
    }

  _dbus_hash_iter_init (table1, &iter);
  while (_dbus_hash_iter_next (&iter))
    {
      const char *key;
      void *value;

      key = _dbus_hash_iter_get_string_key (&iter);
      value = _dbus_hash_iter_get_value (&iter);

      _dbus_assert (_dbus_hash_table_lookup_string (table1, key) == value);

      value = _dbus_strdup ("Different value!");
      if (value == NULL)
        goto out;
      
      _dbus_hash_iter_set_value (&iter, value);

      _dbus_assert (_dbus_hash_table_lookup_string (table1, key) == value);
    }
  
  _dbus_hash_iter_init (table1, &iter);
  while (_dbus_hash_iter_next (&iter))
    {
      _dbus_hash_iter_remove_entry (&iter);
      _dbus_assert (count_entries (table1) == i - 1);
      --i;
    }

  _dbus_hash_iter_init (table2, &iter);
  while (_dbus_hash_iter_next (&iter))
    {
      int key;
      void *value;

      key = _dbus_hash_iter_get_int_key (&iter);
      value = _dbus_hash_iter_get_value (&iter);

      _dbus_assert (_dbus_hash_table_lookup_int (table2, key) == value);

      value = _dbus_strdup ("Different value!");
      if (value == NULL)
        goto out;
      
      _dbus_hash_iter_set_value (&iter, value);

      _dbus_assert (_dbus_hash_table_lookup_int (table2, key) == value);
    }

  i = count_entries (table2);
  _dbus_hash_iter_init (table2, &iter);
  while (_dbus_hash_iter_next (&iter))
    {
      _dbus_hash_iter_remove_entry (&iter);
      _dbus_assert (count_entries (table2) + 1 == i);
      --i;
    }

  /* add/remove interleaved, to check that we grow/shrink the table
   * appropriately
   */
  i = 0;
  while (i < 1000)
    {
      char *key;
      void *value;
            
      key = _dbus_strdup (keys[i]);
      if (key == NULL)
        goto out;

      value = _dbus_strdup ("Value!");
      if (value == NULL)
        goto out;
      
      if (!_dbus_hash_table_insert_string (table1,
                                           key, value))
        goto out;
      
      ++i;
    }

  --i;
  while (i >= 0)
    {
      char *key;
      void *value;      
      
      key = _dbus_strdup (keys[i]);
      if (key == NULL)
        goto out;
      value = _dbus_strdup ("Value!");
      if (value == NULL)
        goto out;

      if (!_dbus_hash_table_remove_string (table1, keys[i]))
        goto out;
      
      if (!_dbus_hash_table_insert_string (table1,
                                           key, value))
        goto out;

      if (!_dbus_hash_table_remove_string (table1, keys[i]))
        goto out;
      
      _dbus_assert (_dbus_hash_table_get_n_entries (table1) == i);
      
      --i;
    }

  /* nuke these tables */
  _dbus_hash_table_unref (table1);
  _dbus_hash_table_unref (table2);


  /* Now do a bunch of things again using _dbus_hash_iter_lookup() to
   * be sure that interface works.
   */
  table1 = _dbus_hash_table_new (DBUS_HASH_STRING,
                                 dbus_free, dbus_free);
  if (table1 == NULL)
    goto out;
  
  table2 = _dbus_hash_table_new (DBUS_HASH_INT,
                                 NULL, dbus_free);
  if (table2 == NULL)
    goto out;
  
  i = 0;
  while (i < 3000)
    {
      void *value;
      char *key;

      key = _dbus_strdup (keys[i]);
      if (key == NULL)
        goto out;
      value = _dbus_strdup ("Value!");
      if (value == NULL)
        goto out;
      
      if (!_dbus_hash_iter_lookup (table1,
                                   key, TRUE, &iter))
        goto out;
      _dbus_assert (_dbus_hash_iter_get_value (&iter) == NULL);
      _dbus_hash_iter_set_value (&iter, value);

      value = _dbus_strdup (keys[i]);
      if (value == NULL)
        goto out;

      if (!_dbus_hash_iter_lookup (table2,
                                   _DBUS_INT_TO_POINTER (i), TRUE, &iter))
        goto out;
      _dbus_assert (_dbus_hash_iter_get_value (&iter) == NULL);
      _dbus_hash_iter_set_value (&iter, value); 
      
      _dbus_assert (count_entries (table1) == i + 1);
      _dbus_assert (count_entries (table2) == i + 1);

      if (!_dbus_hash_iter_lookup (table1, keys[i], FALSE, &iter))
        goto out;
      
      value = _dbus_hash_iter_get_value (&iter);
      _dbus_assert (value != NULL);
      _dbus_assert (strcmp (value, "Value!") == 0);

      /* Iterate just to be sure it works, though
       * it's a stupid thing to do
       */
      while (_dbus_hash_iter_next (&iter))
        ;
      
      if (!_dbus_hash_iter_lookup (table2, _DBUS_INT_TO_POINTER (i), FALSE, &iter))
        goto out;

      value = _dbus_hash_iter_get_value (&iter);
      _dbus_assert (value != NULL);
      _dbus_assert (strcmp (value, keys[i]) == 0);

      /* Iterate just to be sure it works, though
       * it's a stupid thing to do
       */
      while (_dbus_hash_iter_next (&iter))
        ;
      
      ++i;
    }

  --i;
  while (i >= 0)
    {
      if (!_dbus_hash_iter_lookup (table1, keys[i], FALSE, &iter))
        _dbus_assert_not_reached ("hash entry should have existed");
      _dbus_hash_iter_remove_entry (&iter);
      
      if (!_dbus_hash_iter_lookup (table2, _DBUS_INT_TO_POINTER (i), FALSE, &iter))
        _dbus_assert_not_reached ("hash entry should have existed");
      _dbus_hash_iter_remove_entry (&iter);

      _dbus_assert (count_entries (table1) == i);
      _dbus_assert (count_entries (table2) == i);

      --i;
    }

  _dbus_hash_table_unref (table1);
  _dbus_hash_table_unref (table2);

  ret = TRUE;

 out:
  for (i = 0; i < N_HASH_KEYS; i++)
    dbus_free (keys[i]);

  dbus_free (keys);
  
  return ret;
}

#endif /* DBUS_BUILD_TESTS */
