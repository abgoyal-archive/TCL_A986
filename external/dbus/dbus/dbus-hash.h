
/* -*- mode: C; c-file-style: "gnu" -*- */

#ifndef DBUS_HASH_H
#define DBUS_HASH_H

#include <dbus/dbus-memory.h>
#include <dbus/dbus-types.h>

DBUS_BEGIN_DECLS


struct DBusHashIter
{
  void *dummy1; /**< Do not use. */
  void *dummy2; /**< Do not use. */
  void *dummy3; /**< Do not use. */
  void *dummy4; /**< Do not use. */
  int   dummy5; /**< Do not use. */
  int   dummy6; /**< Do not use. */
};

typedef struct DBusHashTable DBusHashTable;
typedef struct DBusHashIter  DBusHashIter;

typedef enum
{
  DBUS_HASH_STRING,        /**< Hash keys are strings. */
  DBUS_HASH_TWO_STRINGS,   /**< Hash key is two strings in one memory block, i.e. foo\\0bar\\0 */
  DBUS_HASH_INT,           /**< Hash keys are integers. */
  DBUS_HASH_POINTER,       /**< Hash keys are pointers. */
  DBUS_HASH_ULONG          /**< Hash keys are unsigned long. */
} DBusHashType;

DBusHashTable* _dbus_hash_table_new                (DBusHashType      type,
                                                    DBusFreeFunction  key_free_function,
                                                    DBusFreeFunction  value_free_function);
DBusHashTable* _dbus_hash_table_ref                (DBusHashTable    *table);
void           _dbus_hash_table_unref              (DBusHashTable    *table);
void           _dbus_hash_table_remove_all         (DBusHashTable    *table);
void           _dbus_hash_iter_init                (DBusHashTable    *table,
                                                    DBusHashIter     *iter);
dbus_bool_t    _dbus_hash_iter_next                (DBusHashIter     *iter);
void           _dbus_hash_iter_remove_entry        (DBusHashIter     *iter);
void*          _dbus_hash_iter_get_value           (DBusHashIter     *iter);
void           _dbus_hash_iter_set_value           (DBusHashIter     *iter,
                                                    void             *value);
int            _dbus_hash_iter_get_int_key         (DBusHashIter     *iter);
const char*    _dbus_hash_iter_get_string_key      (DBusHashIter     *iter);
const char*    _dbus_hash_iter_get_two_strings_key (DBusHashIter     *iter);
unsigned long  _dbus_hash_iter_get_ulong_key       (DBusHashIter     *iter);
dbus_bool_t    _dbus_hash_iter_lookup              (DBusHashTable    *table,
                                                    void             *key,
                                                    dbus_bool_t       create_if_not_found,
                                                    DBusHashIter     *iter);
void*          _dbus_hash_table_lookup_string      (DBusHashTable    *table,
                                                    const char       *key);
void*          _dbus_hash_table_lookup_two_strings (DBusHashTable    *table,
                                                    const char       *key);
void*          _dbus_hash_table_lookup_int         (DBusHashTable    *table,
                                                    int               key);
void*          _dbus_hash_table_lookup_pointer     (DBusHashTable    *table,
                                                    void             *key);
void*          _dbus_hash_table_lookup_ulong       (DBusHashTable    *table,
                                                    unsigned long     key);
dbus_bool_t    _dbus_hash_table_remove_string      (DBusHashTable    *table,
                                                    const char       *key);
dbus_bool_t    _dbus_hash_table_remove_two_strings (DBusHashTable    *table,
                                                    const char       *key);
dbus_bool_t    _dbus_hash_table_remove_int         (DBusHashTable    *table,
                                                    int               key);
dbus_bool_t    _dbus_hash_table_remove_pointer     (DBusHashTable    *table,
                                                    void             *key);
dbus_bool_t    _dbus_hash_table_remove_ulong       (DBusHashTable    *table,
                                                    unsigned long     key);
dbus_bool_t    _dbus_hash_table_insert_string      (DBusHashTable    *table,
                                                    char             *key,
                                                    void             *value);
dbus_bool_t    _dbus_hash_table_insert_two_strings (DBusHashTable    *table,
                                                    char             *key,
                                                    void             *value);
dbus_bool_t    _dbus_hash_table_insert_int         (DBusHashTable    *table,
                                                    int               key,
                                                    void             *value);
dbus_bool_t    _dbus_hash_table_insert_pointer     (DBusHashTable    *table,
                                                    void             *key,
                                                    void             *value);
dbus_bool_t    _dbus_hash_table_insert_ulong       (DBusHashTable    *table,
                                                    unsigned long     key,
                                                    void             *value);
int            _dbus_hash_table_get_n_entries      (DBusHashTable    *table);

/* Preallocation */

/** A preallocated hash entry */
typedef struct DBusPreallocatedHash DBusPreallocatedHash;

DBusPreallocatedHash *_dbus_hash_table_preallocate_entry          (DBusHashTable        *table);
void                  _dbus_hash_table_free_preallocated_entry    (DBusHashTable        *table,
                                                                   DBusPreallocatedHash *preallocated);
void                  _dbus_hash_table_insert_string_preallocated (DBusHashTable        *table,
                                                                   DBusPreallocatedHash *preallocated,
                                                                   char                 *key,
                                                                   void                 *value);

/** @} */

DBUS_END_DECLS

#endif /* DBUS_HASH_H */
