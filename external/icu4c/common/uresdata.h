
#ifndef __RESDATA_H__
#define __RESDATA_H__

#include "unicode/utypes.h"
#include "unicode/udata.h"
#include "unicode/ures.h"
#include "udataswp.h"

typedef enum {
    /** Include a negative value so that the compiler uses the same int type as for UResType. */
    URES_INTERNAL_NONE=-1,

    /** Resource type constant for tables with 32-bit count, key offsets and values. */
    URES_TABLE32=4,

    /**
     * Resource type constant for tables with 16-bit count, key offsets and values.
     * All values are URES_STRING_V2 strings.
     */
    URES_TABLE16=5,

    /** Resource type constant for 16-bit Unicode strings in formatVersion 2. */
    URES_STRING_V2=6,

    /**
     * Resource type constant for arrays with 16-bit count and values.
     * All values are URES_STRING_V2 strings.
     */
    URES_ARRAY16=9
} UResInternalType;

typedef uint32_t Resource;

#define RES_BOGUS 0xffffffff

#define RES_GET_TYPE(res) ((UResType)((res)>>28UL))
#define RES_GET_OFFSET(res) ((res)&0x0fffffff)
#define RES_GET_POINTER(pRoot, res) ((pRoot)+RES_GET_OFFSET(res))

/* get signed and unsigned integer values directly from the Resource handle */
#define RES_GET_INT(res) (((int32_t)((res)<<4L))>>4L)
#define RES_GET_UINT(res) ((res)&0x0fffffff)

#define URES_IS_ARRAY(type) ((type)==URES_ARRAY || (type)==URES_ARRAY16)
#define URES_IS_TABLE(type) ((type)==URES_TABLE || (type)==URES_TABLE16 || (type)==URES_TABLE32)
#define URES_IS_CONTAINER(type) (URES_IS_TABLE(type) || URES_IS_ARRAY(type))

#define URES_MAKE_RESOURCE(type, offset) (((Resource)(type)<<28)|(Resource)(offset))
#define URES_MAKE_EMPTY_RESOURCE(type) ((Resource)(type)<<28)

/* indexes[] value names; indexes are generally 32-bit (Resource) indexes */
enum {
    URES_INDEX_LENGTH,          /* [0] contains URES_INDEX_TOP==the length of indexes[];
                                 *     formatVersion==1: all bits contain the length of indexes[]
                                 *       but the length is much less than 0xff;
                                 *     formatVersion>1:
                                 *       only bits  7..0 contain the length of indexes[],
                                 *            bits 31..8 are reserved and set to 0 */
    URES_INDEX_KEYS_TOP,        /* [1] contains the top of the key strings, */
                                /*     same as the bottom of resources or UTF-16 strings, rounded up */
    URES_INDEX_RESOURCES_TOP,   /* [2] contains the top of all resources */
    URES_INDEX_BUNDLE_TOP,      /* [3] contains the top of the bundle, */
                                /*     in case it were ever different from [2] */
    URES_INDEX_MAX_TABLE_LENGTH,/* [4] max. length of any table */
    URES_INDEX_ATTRIBUTES,      /* [5] attributes bit set, see URES_ATT_* (new in formatVersion 1.2) */
    URES_INDEX_16BIT_TOP,       /* [6] top of the 16-bit units (UTF-16 string v2 UChars, URES_TABLE16, URES_ARRAY16),
                                 *     rounded up (new in formatVersion 2.0, ICU 4.4) */
    URES_INDEX_POOL_CHECKSUM,   /* [7] checksum of the pool bundle (new in formatVersion 2.0, ICU 4.4) */
    URES_INDEX_TOP
};

#define URES_ATT_NO_FALLBACK 1

#define URES_ATT_IS_POOL_BUNDLE 2
#define URES_ATT_USES_POOL_BUNDLE 4


typedef struct {
    UDataMemory *data;
    const int32_t *pRoot;
    const uint16_t *p16BitUnits;
    const char *poolBundleKeys;
    Resource rootRes;
    int32_t localKeyLimit;
    UBool noFallback; /* see URES_ATT_NO_FALLBACK */
    UBool isPoolBundle;
    UBool usesPoolBundle;
    UBool useNativeStrcmp;
} ResourceData;

U_INTERNAL void U_EXPORT2
res_read(ResourceData *pResData,
         const UDataInfo *pInfo, const void *inBytes, int32_t length,
         UErrorCode *errorCode);

U_CFUNC void
res_load(ResourceData *pResData,
         const char *path, const char *name, UErrorCode *errorCode);

U_CFUNC void
res_unload(ResourceData *pResData);

U_INTERNAL UResType U_EXPORT2
res_getPublicType(Resource res);

U_INTERNAL const UChar * U_EXPORT2
res_getString(const ResourceData *pResData, Resource res, int32_t *pLength);

U_INTERNAL const UChar * U_EXPORT2
res_getAlias(const ResourceData *pResData, Resource res, int32_t *pLength);

U_INTERNAL const uint8_t * U_EXPORT2
res_getBinary(const ResourceData *pResData, Resource res, int32_t *pLength);

U_INTERNAL const int32_t * U_EXPORT2
res_getIntVector(const ResourceData *pResData, Resource res, int32_t *pLength);

U_INTERNAL Resource U_EXPORT2
res_getResource(const ResourceData *pResData, const char *key);

U_INTERNAL int32_t U_EXPORT2
res_countArrayItems(const ResourceData *pResData, Resource res);

U_INTERNAL Resource U_EXPORT2
res_getArrayItem(const ResourceData *pResData, Resource array, int32_t indexS);

U_INTERNAL Resource U_EXPORT2
res_getTableItemByIndex(const ResourceData *pResData, Resource table, int32_t indexS, const char ** key);

U_INTERNAL Resource U_EXPORT2
res_getTableItemByKey(const ResourceData *pResData, Resource table, int32_t *indexS, const char* * key);

U_CFUNC Resource res_findResource(const ResourceData *pResData, Resource r, char** path, const char** key);

U_CAPI int32_t U_EXPORT2
ures_swap(const UDataSwapper *ds,
          const void *inData, int32_t length, void *outData,
          UErrorCode *pErrorCode);

#endif
