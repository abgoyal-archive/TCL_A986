
#ifndef UHASH_H
#define UHASH_H

#include "unicode/utypes.h"



U_CDECL_BEGIN

union UHashTok {
    void*   pointer;
    int32_t integer;
};
typedef union UHashTok UHashTok;

struct UHashElement {
    /* Reorder these elements to pack nicely if necessary */
    int32_t  hashcode;
    UHashTok value;
    UHashTok key;
};
typedef struct UHashElement UHashElement;

typedef int32_t U_CALLCONV UHashFunction(const UHashTok key);

typedef UBool U_CALLCONV UKeyComparator(const UHashTok key1,
                                        const UHashTok key2);
typedef UBool U_CALLCONV UValueComparator(const UHashTok val1,
                                          const UHashTok val2);
typedef void U_CALLCONV UObjectDeleter(void* obj);

enum UHashResizePolicy {
    U_GROW,            /* Grow on demand, do not shrink */
    U_GROW_AND_SHRINK, /* Grow and shrink on demand */
    U_FIXED            /* Never change size */
};

struct UHashtable {

    /* Main key-value pair storage array */

    UHashElement *elements;

    /* Function pointers */

    UHashFunction *keyHasher;      /* Computes hash from key.
                                   * Never null. */
    UKeyComparator *keyComparator; /* Compares keys for equality.
                                   * Never null. */
    UValueComparator *valueComparator; /* Compares the values for equality */

    UObjectDeleter *keyDeleter;    /* Deletes keys when required.
                                   * If NULL won't do anything */
    UObjectDeleter *valueDeleter;  /* Deletes values when required.
                                   * If NULL won't do anything */

    /* Size parameters */
  
    int32_t     count;      /* The number of key-value pairs in this table.
                             * 0 <= count <= length.  In practice we
                             * never let count == length (see code). */
    int32_t     length;     /* The physical size of the arrays hashes, keys
                             * and values.  Must be prime. */

    /* Rehashing thresholds */
    
    int32_t     highWaterMark;  /* If count > highWaterMark, rehash */
    int32_t     lowWaterMark;   /* If count < lowWaterMark, rehash */
    float       highWaterRatio; /* 0..1; high water as a fraction of length */
    float       lowWaterRatio;  /* 0..1; low water as a fraction of length */
    
    int8_t      primeIndex;     /* Index into our prime table for length.
                                 * length == PRIMES[primeIndex] */
    UBool       allocated; /* Was this UHashtable allocated? */
};
typedef struct UHashtable UHashtable;

U_CDECL_END


U_CAPI UHashtable* U_EXPORT2 
uhash_open(UHashFunction *keyHash,
           UKeyComparator *keyComp,
           UValueComparator *valueComp,
           UErrorCode *status);

U_CAPI UHashtable* U_EXPORT2 
uhash_openSize(UHashFunction *keyHash,
               UKeyComparator *keyComp,
               UValueComparator *valueComp,
               int32_t size,
               UErrorCode *status);

U_CAPI UHashtable* U_EXPORT2 
uhash_init(UHashtable *hash,
           UHashFunction *keyHash,
           UKeyComparator *keyComp,
           UValueComparator *valueComp,
           UErrorCode *status);

U_CAPI void U_EXPORT2 
uhash_close(UHashtable *hash);



U_CAPI UHashFunction *U_EXPORT2 
uhash_setKeyHasher(UHashtable *hash, UHashFunction *fn);

U_CAPI UKeyComparator *U_EXPORT2 
uhash_setKeyComparator(UHashtable *hash, UKeyComparator *fn);

U_CAPI UValueComparator *U_EXPORT2 
uhash_setValueComparator(UHashtable *hash, UValueComparator *fn);

U_CAPI UObjectDeleter *U_EXPORT2 
uhash_setKeyDeleter(UHashtable *hash, UObjectDeleter *fn);

U_CAPI UObjectDeleter *U_EXPORT2 
uhash_setValueDeleter(UHashtable *hash, UObjectDeleter *fn);

U_CAPI void U_EXPORT2 
uhash_setResizePolicy(UHashtable *hash, enum UHashResizePolicy policy);

U_CAPI int32_t U_EXPORT2 
uhash_count(const UHashtable *hash);

U_CAPI void* U_EXPORT2 
uhash_put(UHashtable *hash,
          void *key,
          void *value,
          UErrorCode *status);

U_CAPI void* U_EXPORT2 
uhash_iput(UHashtable *hash,
           int32_t key,
           void* value,
           UErrorCode *status);

U_CAPI int32_t U_EXPORT2 
uhash_puti(UHashtable *hash,
           void* key,
           int32_t value,
           UErrorCode *status);

U_CAPI int32_t U_EXPORT2 
uhash_iputi(UHashtable *hash,
           int32_t key,
           int32_t value,
           UErrorCode *status);

U_CAPI void* U_EXPORT2 
uhash_get(const UHashtable *hash, 
          const void *key);

U_CAPI void* U_EXPORT2 
uhash_iget(const UHashtable *hash,
           int32_t key);

U_CAPI int32_t U_EXPORT2 
uhash_geti(const UHashtable *hash,
           const void* key);
U_CAPI int32_t U_EXPORT2 
uhash_igeti(const UHashtable *hash,
           int32_t key);

U_CAPI void* U_EXPORT2 
uhash_remove(UHashtable *hash,
             const void *key);

U_CAPI void* U_EXPORT2 
uhash_iremove(UHashtable *hash,
              int32_t key);

U_CAPI int32_t U_EXPORT2 
uhash_removei(UHashtable *hash,
              const void* key);

U_CAPI int32_t U_EXPORT2 
uhash_iremovei(UHashtable *hash,
               int32_t key);

U_CAPI void U_EXPORT2 
uhash_removeAll(UHashtable *hash);

U_CAPI const UHashElement* U_EXPORT2 
uhash_find(const UHashtable *hash, const void* key);

U_CAPI const UHashElement* U_EXPORT2 
uhash_nextElement(const UHashtable *hash,
                  int32_t *pos);

U_CAPI void* U_EXPORT2 
uhash_removeElement(UHashtable *hash, const UHashElement* e);





U_CAPI int32_t U_EXPORT2 
uhash_hashUChars(const UHashTok key);

U_CAPI int32_t U_EXPORT2 
uhash_hashChars(const UHashTok key);

/* Used by UnicodeString to compute its hashcode - Not public API. */
U_CAPI int32_t U_EXPORT2 
uhash_hashUCharsN(const UChar *key, int32_t length);

U_CAPI int32_t U_EXPORT2
uhash_hashIChars(const UHashTok key);

U_CAPI UBool U_EXPORT2 
uhash_compareUChars(const UHashTok key1, const UHashTok key2);

U_CAPI UBool U_EXPORT2 
uhash_compareChars(const UHashTok key1, const UHashTok key2);

U_CAPI UBool U_EXPORT2 
uhash_compareIChars(const UHashTok key1, const UHashTok key2);


U_CAPI int32_t U_EXPORT2 
uhash_hashUnicodeString(const UHashTok key);

U_CAPI int32_t U_EXPORT2 
uhash_hashCaselessUnicodeString(const UHashTok key);

U_CAPI UBool U_EXPORT2 
uhash_compareUnicodeString(const UHashTok key1, const UHashTok key2);

U_CAPI UBool U_EXPORT2 
uhash_compareCaselessUnicodeString(const UHashTok key1, const UHashTok key2);

U_CAPI void U_EXPORT2 
uhash_deleteUnicodeString(void *obj);


U_CAPI int32_t U_EXPORT2 
uhash_hashLong(const UHashTok key);

U_CAPI UBool U_EXPORT2 
uhash_compareLong(const UHashTok key1, const UHashTok key2);


U_CAPI void U_EXPORT2 
uhash_deleteHashtable(void *obj);

U_CAPI void U_EXPORT2 
uhash_deleteUVector(void *obj);

U_CAPI void U_EXPORT2 
uhash_freeBlock(void *obj);

U_CAPI UBool U_EXPORT2 
uhash_equals(const UHashtable* hash1, const UHashtable* hash2);

#endif
