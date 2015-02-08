
#include "uhash.h"
#include "unicode/ustring.h"
#include "cstring.h"
#include "cmemory.h"
#include "uassert.h"



static const int32_t PRIMES[] = {
    13, 31, 61, 127, 251, 509, 1021, 2039, 4093, 8191, 16381, 32749,
    65521, 131071, 262139, 524287, 1048573, 2097143, 4194301, 8388593,
    16777213, 33554393, 67108859, 134217689, 268435399, 536870909,
    1073741789, 2147483647 /*, 4294967291 */
};

#define PRIMES_LENGTH (sizeof(PRIMES) / sizeof(PRIMES[0]))
#define DEFAULT_PRIME_INDEX 3

static const float RESIZE_POLICY_RATIO_TABLE[6] = {
    /* low, high water ratio */
    0.0F, 0.5F, /* U_GROW: Grow on demand, do not shrink */
    0.1F, 0.5F, /* U_GROW_AND_SHRINK: Grow and shrink on demand */
    0.0F, 1.0F  /* U_FIXED: Never change size */
};

#define HASH_DELETED    ((int32_t) 0x80000000)
#define HASH_EMPTY      ((int32_t) HASH_DELETED + 1)

#define IS_EMPTY_OR_DELETED(x) ((x) < 0)

#define HASH_DELETE_KEY_VALUE(hash, keypointer, valuepointer) \
            if (hash->keyDeleter != NULL && keypointer != NULL) { \
                (*hash->keyDeleter)(keypointer); \
            } \
            if (hash->valueDeleter != NULL && valuepointer != NULL) { \
                (*hash->valueDeleter)(valuepointer); \
            }

#define HINT_KEY_POINTER   (1)
#define HINT_VALUE_POINTER (2)


static UHashTok
_uhash_setElement(UHashtable *hash, UHashElement* e,
                  int32_t hashcode,
                  UHashTok key, UHashTok value, int8_t hint) {

    UHashTok oldValue = e->value;
    if (hash->keyDeleter != NULL && e->key.pointer != NULL &&
        e->key.pointer != key.pointer) { /* Avoid double deletion */
        (*hash->keyDeleter)(e->key.pointer);
    }
    if (hash->valueDeleter != NULL) {
        if (oldValue.pointer != NULL &&
            oldValue.pointer != value.pointer) { /* Avoid double deletion */
            (*hash->valueDeleter)(oldValue.pointer);
        }
        oldValue.pointer = NULL;
    }
    /* Compilers should copy the UHashTok union correctly, but even if
     * they do, memory heap tools (e.g. BoundsChecker) can get
     * confused when a pointer is cloaked in a union and then copied.
     * TO ALLEVIATE THIS, we use hints (based on what API the user is
     * calling) to copy pointers when we know the user thinks
     * something is a pointer. */
    if (hint & HINT_KEY_POINTER) {
        e->key.pointer = key.pointer;
    } else {
        e->key = key;
    }
    if (hint & HINT_VALUE_POINTER) {
        e->value.pointer = value.pointer;
    } else {
        e->value = value;
    }
    e->hashcode = hashcode;
    return oldValue;
}

static UHashTok
_uhash_internalRemoveElement(UHashtable *hash, UHashElement* e) {
    UHashTok empty;
    U_ASSERT(!IS_EMPTY_OR_DELETED(e->hashcode));
    --hash->count;
    empty.pointer = NULL; empty.integer = 0;
    return _uhash_setElement(hash, e, HASH_DELETED, empty, empty, 0);
}

static void
_uhash_internalSetResizePolicy(UHashtable *hash, enum UHashResizePolicy policy) {
    U_ASSERT(hash != NULL);
    U_ASSERT(((int32_t)policy) >= 0);
    U_ASSERT(((int32_t)policy) < 3);
    hash->lowWaterRatio  = RESIZE_POLICY_RATIO_TABLE[policy * 2];
    hash->highWaterRatio = RESIZE_POLICY_RATIO_TABLE[policy * 2 + 1];
}

static void
_uhash_allocate(UHashtable *hash,
                int32_t primeIndex,
                UErrorCode *status) {

    UHashElement *p, *limit;
    UHashTok emptytok;

    if (U_FAILURE(*status)) return;

    U_ASSERT(primeIndex >= 0 && primeIndex < PRIMES_LENGTH);

    hash->primeIndex = primeIndex;
    hash->length = PRIMES[primeIndex];

    p = hash->elements = (UHashElement*)
        uprv_malloc(sizeof(UHashElement) * hash->length);

    if (hash->elements == NULL) {
        *status = U_MEMORY_ALLOCATION_ERROR;
        return;
    }

    emptytok.pointer = NULL; /* Only one of these two is needed */
    emptytok.integer = 0;    /* but we don't know which one. */
    
    limit = p + hash->length;
    while (p < limit) {
        p->key = emptytok;
        p->value = emptytok;
        p->hashcode = HASH_EMPTY;
        ++p;
    }

    hash->count = 0;
    hash->lowWaterMark = (int32_t)(hash->length * hash->lowWaterRatio);
    hash->highWaterMark = (int32_t)(hash->length * hash->highWaterRatio);
}

static UHashtable*
_uhash_init(UHashtable *result,
              UHashFunction *keyHash, 
              UKeyComparator *keyComp,
              UValueComparator *valueComp,
              int32_t primeIndex,
              UErrorCode *status)
{
    if (U_FAILURE(*status)) return NULL;
    U_ASSERT(keyHash != NULL);
    U_ASSERT(keyComp != NULL);

    result->keyHasher       = keyHash;
    result->keyComparator   = keyComp;
    result->valueComparator = valueComp;
    result->keyDeleter      = NULL;
    result->valueDeleter    = NULL;
    result->allocated       = FALSE;
    _uhash_internalSetResizePolicy(result, U_GROW);

    _uhash_allocate(result, primeIndex, status);

    if (U_FAILURE(*status)) {
        return NULL;
    }

    return result;
}

static UHashtable*
_uhash_create(UHashFunction *keyHash, 
              UKeyComparator *keyComp,
              UValueComparator *valueComp,
              int32_t primeIndex,
              UErrorCode *status) {
    UHashtable *result;

    if (U_FAILURE(*status)) return NULL;

    result = (UHashtable*) uprv_malloc(sizeof(UHashtable));
    if (result == NULL) {
        *status = U_MEMORY_ALLOCATION_ERROR;
        return NULL;
    }

    _uhash_init(result, keyHash, keyComp, valueComp, primeIndex, status);
    result->allocated       = TRUE;

    if (U_FAILURE(*status)) {
        uprv_free(result);
        return NULL;
    }

    return result;
}

static UHashElement*
_uhash_find(const UHashtable *hash, UHashTok key,
            int32_t hashcode) {

    int32_t firstDeleted = -1;  /* assume invalid index */
    int32_t theIndex, startIndex;
    int32_t jump = 0; /* lazy evaluate */
    int32_t tableHash;
    UHashElement *elements = hash->elements;

    hashcode &= 0x7FFFFFFF; /* must be positive */
    startIndex = theIndex = (hashcode ^ 0x4000000) % hash->length;

    do {
        tableHash = elements[theIndex].hashcode;
        if (tableHash == hashcode) {          /* quick check */
            if ((*hash->keyComparator)(key, elements[theIndex].key)) {
                return &(elements[theIndex]);
            }
        } else if (!IS_EMPTY_OR_DELETED(tableHash)) {
            /* We have hit a slot which contains a key-value pair,
             * but for which the hash code does not match.  Keep
             * looking.
             */
        } else if (tableHash == HASH_EMPTY) { /* empty, end o' the line */
            break;
        } else if (firstDeleted < 0) { /* remember first deleted */
            firstDeleted = theIndex;
        }
        if (jump == 0) { /* lazy compute jump */
            /* The jump value must be relatively prime to the table
             * length.  As long as the length is prime, then any value
             * 1..length-1 will be relatively prime to it.
             */
            jump = (hashcode % (hash->length - 1)) + 1;
        }
        theIndex = (theIndex + jump) % hash->length;
    } while (theIndex != startIndex);

    if (firstDeleted >= 0) {
        theIndex = firstDeleted; /* reset if had deleted slot */
    } else if (tableHash != HASH_EMPTY) {
        /* We get to this point if the hashtable is full (no empty or
         * deleted slots), and we've failed to find a match.  THIS
         * WILL NEVER HAPPEN as long as uhash_put() makes sure that
         * count is always < length.
         */
        U_ASSERT(FALSE);
        return NULL; /* Never happens if uhash_put() behaves */
    }
    return &(elements[theIndex]);
}

static void
_uhash_rehash(UHashtable *hash, UErrorCode *status) {

    UHashElement *old = hash->elements;
    int32_t oldLength = hash->length;
    int32_t newPrimeIndex = hash->primeIndex;
    int32_t i;

    if (hash->count > hash->highWaterMark) {
        if (++newPrimeIndex >= PRIMES_LENGTH) {
            return;
        }
    } else if (hash->count < hash->lowWaterMark) {
        if (--newPrimeIndex < 0) {
            return;
        }
    } else {
        return;
    }

    _uhash_allocate(hash, newPrimeIndex, status);

    if (U_FAILURE(*status)) {
        hash->elements = old;
        hash->length = oldLength;       
        return;
    }

    for (i = oldLength - 1; i >= 0; --i) {
        if (!IS_EMPTY_OR_DELETED(old[i].hashcode)) {
            UHashElement *e = _uhash_find(hash, old[i].key, old[i].hashcode);
            U_ASSERT(e != NULL);
            U_ASSERT(e->hashcode == HASH_EMPTY);
            e->key = old[i].key;
            e->value = old[i].value;
            e->hashcode = old[i].hashcode;
            ++hash->count;
        }
    }

    uprv_free(old);
}

static UHashTok
_uhash_remove(UHashtable *hash,
              UHashTok key) {
    /* First find the position of the key in the table.  If the object
     * has not been removed already, remove it.  If the user wanted
     * keys deleted, then delete it also.  We have to put a special
     * hashcode in that position that means that something has been
     * deleted, since when we do a find, we have to continue PAST any
     * deleted values.
     */
    UHashTok result;
    UHashElement* e = _uhash_find(hash, key, hash->keyHasher(key));
    U_ASSERT(e != NULL);
    result.pointer = NULL;
    result.integer = 0;
    if (!IS_EMPTY_OR_DELETED(e->hashcode)) {
        result = _uhash_internalRemoveElement(hash, e);
        if (hash->count < hash->lowWaterMark) {
            UErrorCode status = U_ZERO_ERROR;
            _uhash_rehash(hash, &status);
        }
    }
    return result;
}

static UHashTok
_uhash_put(UHashtable *hash,
           UHashTok key,
           UHashTok value,
           int8_t hint,
           UErrorCode *status) {

    /* Put finds the position in the table for the new value.  If the
     * key is already in the table, it is deleted, if there is a
     * non-NULL keyDeleter.  Then the key, the hash and the value are
     * all put at the position in their respective arrays.
     */
    int32_t hashcode;
    UHashElement* e;
    UHashTok emptytok;

    if (U_FAILURE(*status)) {
        goto err;
    }
    U_ASSERT(hash != NULL);
    /* Cannot always check pointer here or iSeries sees NULL every time. */
    if ((hint & HINT_VALUE_POINTER) && value.pointer == NULL) {
        /* Disallow storage of NULL values, since NULL is returned by
         * get() to indicate an absent key.  Storing NULL == removing.
         */
        return _uhash_remove(hash, key);
    }
    if (hash->count > hash->highWaterMark) {
        _uhash_rehash(hash, status);
        if (U_FAILURE(*status)) {
            goto err;
        }
    }

    hashcode = (*hash->keyHasher)(key);
    e = _uhash_find(hash, key, hashcode);
    U_ASSERT(e != NULL);

    if (IS_EMPTY_OR_DELETED(e->hashcode)) {
        /* Important: We must never actually fill the table up.  If we
         * do so, then _uhash_find() will return NULL, and we'll have
         * to check for NULL after every call to _uhash_find().  To
         * avoid this we make sure there is always at least one empty
         * or deleted slot in the table.  This only is a problem if we
         * are out of memory and rehash isn't working.
         */
        ++hash->count;
        if (hash->count == hash->length) {
            /* Don't allow count to reach length */
            --hash->count;
            *status = U_MEMORY_ALLOCATION_ERROR;
            goto err;
        }
    }

    /* We must in all cases handle storage properly.  If there was an
     * old key, then it must be deleted (if the deleter != NULL).
     * Make hashcodes stored in table positive.
     */
    return _uhash_setElement(hash, e, hashcode & 0x7FFFFFFF, key, value, hint);

 err:
    /* If the deleters are non-NULL, this method adopts its key and/or
     * value arguments, and we must be sure to delete the key and/or
     * value in all cases, even upon failure.
     */
    HASH_DELETE_KEY_VALUE(hash, key.pointer, value.pointer);
    emptytok.pointer = NULL; emptytok.integer = 0;
    return emptytok;
}



U_CAPI UHashtable* U_EXPORT2
uhash_open(UHashFunction *keyHash, 
           UKeyComparator *keyComp,
           UValueComparator *valueComp,
           UErrorCode *status) {

    return _uhash_create(keyHash, keyComp, valueComp, DEFAULT_PRIME_INDEX, status);
}

U_CAPI UHashtable* U_EXPORT2
uhash_openSize(UHashFunction *keyHash, 
               UKeyComparator *keyComp,
               UValueComparator *valueComp,
               int32_t size,
               UErrorCode *status) {

    /* Find the smallest index i for which PRIMES[i] >= size. */
    int32_t i = 0;
    while (i<(PRIMES_LENGTH-1) && PRIMES[i]<size) {
        ++i;
    }

    return _uhash_create(keyHash, keyComp, valueComp, i, status);
}

U_CAPI UHashtable* U_EXPORT2
uhash_init(UHashtable *fillinResult,
           UHashFunction *keyHash, 
           UKeyComparator *keyComp,
           UValueComparator *valueComp,
           UErrorCode *status) {

    return _uhash_init(fillinResult, keyHash, keyComp, valueComp, DEFAULT_PRIME_INDEX, status);
}

U_CAPI void U_EXPORT2
uhash_close(UHashtable *hash) {
    if (hash == NULL) {
        return;
    }
    if (hash->elements != NULL) {
        if (hash->keyDeleter != NULL || hash->valueDeleter != NULL) {
            int32_t pos=-1;
            UHashElement *e;
            while ((e = (UHashElement*) uhash_nextElement(hash, &pos)) != NULL) {
                HASH_DELETE_KEY_VALUE(hash, e->key.pointer, e->value.pointer);
            }
        }
        uprv_free(hash->elements);
        hash->elements = NULL;
    }
    if (hash->allocated) {
        uprv_free(hash);
    }
}

U_CAPI UHashFunction *U_EXPORT2
uhash_setKeyHasher(UHashtable *hash, UHashFunction *fn) {
    UHashFunction *result = hash->keyHasher;
    hash->keyHasher = fn;
    return result;
}

U_CAPI UKeyComparator *U_EXPORT2
uhash_setKeyComparator(UHashtable *hash, UKeyComparator *fn) {
    UKeyComparator *result = hash->keyComparator;
    hash->keyComparator = fn;
    return result;
}
U_CAPI UValueComparator *U_EXPORT2 
uhash_setValueComparator(UHashtable *hash, UValueComparator *fn){
    UValueComparator *result = hash->valueComparator;
    hash->valueComparator = fn;
    return result;
}

U_CAPI UObjectDeleter *U_EXPORT2
uhash_setKeyDeleter(UHashtable *hash, UObjectDeleter *fn) {
    UObjectDeleter *result = hash->keyDeleter;
    hash->keyDeleter = fn;
    return result;
}

U_CAPI UObjectDeleter *U_EXPORT2
uhash_setValueDeleter(UHashtable *hash, UObjectDeleter *fn) {
    UObjectDeleter *result = hash->valueDeleter;
    hash->valueDeleter = fn;
    return result;
}

U_CAPI void U_EXPORT2
uhash_setResizePolicy(UHashtable *hash, enum UHashResizePolicy policy) {
    UErrorCode status = U_ZERO_ERROR;
    _uhash_internalSetResizePolicy(hash, policy);
    hash->lowWaterMark  = (int32_t)(hash->length * hash->lowWaterRatio);
    hash->highWaterMark = (int32_t)(hash->length * hash->highWaterRatio);    
    _uhash_rehash(hash, &status);
}

U_CAPI int32_t U_EXPORT2
uhash_count(const UHashtable *hash) {
    return hash->count;
}

U_CAPI void* U_EXPORT2
uhash_get(const UHashtable *hash,
          const void* key) {
    UHashTok keyholder;
    keyholder.pointer = (void*) key;
    return _uhash_find(hash, keyholder, hash->keyHasher(keyholder))->value.pointer;
}

U_CAPI void* U_EXPORT2
uhash_iget(const UHashtable *hash,
           int32_t key) {
    UHashTok keyholder;
    keyholder.integer = key;
    return _uhash_find(hash, keyholder, hash->keyHasher(keyholder))->value.pointer;
}

U_CAPI int32_t U_EXPORT2
uhash_geti(const UHashtable *hash,
           const void* key) {
    UHashTok keyholder;
    keyholder.pointer = (void*) key;
    return _uhash_find(hash, keyholder, hash->keyHasher(keyholder))->value.integer;
}

U_CAPI int32_t U_EXPORT2
uhash_igeti(const UHashtable *hash,
           int32_t key) {
    UHashTok keyholder;
    keyholder.integer = key;
    return _uhash_find(hash, keyholder, hash->keyHasher(keyholder))->value.integer;
}

U_CAPI void* U_EXPORT2
uhash_put(UHashtable *hash,
          void* key,
          void* value,
          UErrorCode *status) {
    UHashTok keyholder, valueholder;
    keyholder.pointer = key;
    valueholder.pointer = value;
    return _uhash_put(hash, keyholder, valueholder,
                      HINT_KEY_POINTER | HINT_VALUE_POINTER,
                      status).pointer;
}

U_CAPI void* U_EXPORT2
uhash_iput(UHashtable *hash,
           int32_t key,
           void* value,
           UErrorCode *status) {
    UHashTok keyholder, valueholder;
    keyholder.integer = key;
    valueholder.pointer = value;
    return _uhash_put(hash, keyholder, valueholder,
                      HINT_VALUE_POINTER,
                      status).pointer;
}

U_CAPI int32_t U_EXPORT2
uhash_puti(UHashtable *hash,
           void* key,
           int32_t value,
           UErrorCode *status) {
    UHashTok keyholder, valueholder;
    keyholder.pointer = key;
    valueholder.integer = value;
    return _uhash_put(hash, keyholder, valueholder,
                      HINT_KEY_POINTER,
                      status).integer;
}


U_CAPI int32_t U_EXPORT2
uhash_iputi(UHashtable *hash,
           int32_t key,
           int32_t value,
           UErrorCode *status) {
    UHashTok keyholder, valueholder;
    keyholder.integer = key;
    valueholder.integer = value;
    return _uhash_put(hash, keyholder, valueholder,
                      0, /* neither is a ptr */
                      status).integer;
}

U_CAPI void* U_EXPORT2
uhash_remove(UHashtable *hash,
             const void* key) {
    UHashTok keyholder;
    keyholder.pointer = (void*) key;
    return _uhash_remove(hash, keyholder).pointer;
}

U_CAPI void* U_EXPORT2
uhash_iremove(UHashtable *hash,
              int32_t key) {
    UHashTok keyholder;
    keyholder.integer = key;
    return _uhash_remove(hash, keyholder).pointer;
}

U_CAPI int32_t U_EXPORT2
uhash_removei(UHashtable *hash,
              const void* key) {
    UHashTok keyholder;
    keyholder.pointer = (void*) key;
    return _uhash_remove(hash, keyholder).integer;
}

U_CAPI int32_t U_EXPORT2
uhash_iremovei(UHashtable *hash,
               int32_t key) {
    UHashTok keyholder;
    keyholder.integer = key;
    return _uhash_remove(hash, keyholder).integer;
}

U_CAPI void U_EXPORT2
uhash_removeAll(UHashtable *hash) {
    int32_t pos = -1;
    const UHashElement *e;
    U_ASSERT(hash != NULL);
    if (hash->count != 0) {
        while ((e = uhash_nextElement(hash, &pos)) != NULL) {
            uhash_removeElement(hash, e);
        }
    }
    U_ASSERT(hash->count == 0);
}

U_CAPI const UHashElement* U_EXPORT2
uhash_find(const UHashtable *hash, const void* key) {
    UHashTok keyholder;
    const UHashElement *e;
    keyholder.pointer = (void*) key;
    e = _uhash_find(hash, keyholder, hash->keyHasher(keyholder));
    return IS_EMPTY_OR_DELETED(e->hashcode) ? NULL : e;
}

U_CAPI const UHashElement* U_EXPORT2
uhash_nextElement(const UHashtable *hash, int32_t *pos) {
    /* Walk through the array until we find an element that is not
     * EMPTY and not DELETED.
     */
    int32_t i;
    U_ASSERT(hash != NULL);
    for (i = *pos + 1; i < hash->length; ++i) {
        if (!IS_EMPTY_OR_DELETED(hash->elements[i].hashcode)) {
            *pos = i;
            return &(hash->elements[i]);
        }
    }

    /* No more elements */
    return NULL;
}

U_CAPI void* U_EXPORT2
uhash_removeElement(UHashtable *hash, const UHashElement* e) {
    U_ASSERT(hash != NULL);
    U_ASSERT(e != NULL);
    if (!IS_EMPTY_OR_DELETED(e->hashcode)) {
        UHashElement *nce = (UHashElement *)e;
        return _uhash_internalRemoveElement(hash, nce).pointer;
    }
    return NULL;
}






#define STRING_HASH(TYPE, STR, STRLEN, DEREF) \
    int32_t hash = 0;                         \
    const TYPE *p = (const TYPE*) STR;        \
    if (p != NULL) {                          \
        int32_t len = (int32_t)(STRLEN);      \
        int32_t inc = ((len - 32) / 32) + 1;  \
        const TYPE *limit = p + len;          \
        while (p<limit) {                     \
            hash = (hash * 37) + DEREF;       \
            p += inc;                         \
        }                                     \
    }                                         \
    return hash

U_CAPI int32_t U_EXPORT2
uhash_hashUChars(const UHashTok key) {
    STRING_HASH(UChar, key.pointer, u_strlen(p), *p);
}

/* Used by UnicodeString to compute its hashcode - Not public API. */
U_CAPI int32_t U_EXPORT2
uhash_hashUCharsN(const UChar *str, int32_t length) {
    STRING_HASH(UChar, str, length, *p);
}

U_CAPI int32_t U_EXPORT2
uhash_hashChars(const UHashTok key) {
    STRING_HASH(uint8_t, key.pointer, uprv_strlen((char*)p), *p);
}

U_CAPI int32_t U_EXPORT2
uhash_hashIChars(const UHashTok key) {
    STRING_HASH(uint8_t, key.pointer, uprv_strlen((char*)p), uprv_tolower(*p));
}

U_CAPI UBool U_EXPORT2 
uhash_equals(const UHashtable* hash1, const UHashtable* hash2){
    
    int32_t count1, count2, pos, i;

    if(hash1==hash2){
        return TRUE;
    }

    /*
     * Make sure that we are comparing 2 valid hashes of the same type
     * with valid comparison functions.
     * Without valid comparison functions, a binary comparison
     * of the hash values will yield random results on machines
     * with 64-bit pointers and 32-bit integer hashes.
     * A valueComparator is normally optional.
     */
    if (hash1==NULL || hash2==NULL ||
        hash1->keyComparator != hash2->keyComparator ||
        hash1->valueComparator != hash2->valueComparator ||
        hash1->valueComparator == NULL)
    {
        /*
        Normally we would return an error here about incompatible hash tables,
        but we return FALSE instead.
        */
        return FALSE;
    }

    count1 = uhash_count(hash1);
    count2 = uhash_count(hash2);
    if(count1!=count2){
        return FALSE;
    }
    
    pos=-1;
    for(i=0; i<count1; i++){
        const UHashElement* elem1 = uhash_nextElement(hash1, &pos);
        const UHashTok key1 = elem1->key;
        const UHashTok val1 = elem1->value;
        /* here the keys are not compared, instead the key form hash1 is used to fetch
         * value from hash2. If the hashes are equal then then both hashes should 
         * contain equal values for the same key!
         */
        const UHashElement* elem2 = _uhash_find(hash2, key1, hash2->keyHasher(key1));
        const UHashTok val2 = elem2->value;
        if(hash1->valueComparator(val1, val2)==FALSE){
            return FALSE;
        }
    }
    return TRUE;
}


U_CAPI UBool U_EXPORT2
uhash_compareUChars(const UHashTok key1, const UHashTok key2) {
    const UChar *p1 = (const UChar*) key1.pointer;
    const UChar *p2 = (const UChar*) key2.pointer;
    if (p1 == p2) {
        return TRUE;
    }
    if (p1 == NULL || p2 == NULL) {
        return FALSE;
    }
    while (*p1 != 0 && *p1 == *p2) {
        ++p1;
        ++p2;
    }
    return (UBool)(*p1 == *p2);
}

U_CAPI UBool U_EXPORT2
uhash_compareChars(const UHashTok key1, const UHashTok key2) {
    const char *p1 = (const char*) key1.pointer;
    const char *p2 = (const char*) key2.pointer;
    if (p1 == p2) {
        return TRUE;
    }
    if (p1 == NULL || p2 == NULL) {
        return FALSE;
    }
    while (*p1 != 0 && *p1 == *p2) {
        ++p1;
        ++p2;
    }
    return (UBool)(*p1 == *p2);
}

U_CAPI UBool U_EXPORT2
uhash_compareIChars(const UHashTok key1, const UHashTok key2) {
    const char *p1 = (const char*) key1.pointer;
    const char *p2 = (const char*) key2.pointer;
    if (p1 == p2) {
        return TRUE;
    }
    if (p1 == NULL || p2 == NULL) {
        return FALSE;
    }
    while (*p1 != 0 && uprv_tolower(*p1) == uprv_tolower(*p2)) {
        ++p1;
        ++p2;
    }
    return (UBool)(*p1 == *p2);
}


U_CAPI int32_t U_EXPORT2
uhash_hashLong(const UHashTok key) {
    return key.integer;
}

U_CAPI UBool U_EXPORT2
uhash_compareLong(const UHashTok key1, const UHashTok key2) {
    return (UBool)(key1.integer == key2.integer);
}


U_CAPI void U_EXPORT2
uhash_freeBlock(void *obj) {
    uprv_free(obj);
}

