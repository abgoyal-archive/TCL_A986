
#ifndef __RESDATA_H__
#define __RESDATA_H__

#include "unicode/utypes.h"
#include "unicode/udata.h"
#include "udataswp.h"

typedef uint32_t Resource;

#define RES_BOGUS 0xffffffff

#define RES_GET_TYPE(res) ((UResType)((res)>>28UL))
#define RES_GET_OFFSET(res) ((res)&0x0fffffff)
#define RES_GET_POINTER(pRoot, res) ((pRoot)+RES_GET_OFFSET(res))

/* get signed and unsigned integer values directly from the Resource handle */
#define RES_GET_INT(res) (((int32_t)((res)<<4L))>>4L)
#define RES_GET_UINT(res) ((res)&0x0fffffff)

/* indexes[] value names; indexes are generally 32-bit (Resource) indexes */
enum {
    URES_INDEX_LENGTH,          /* [0] contains URES_INDEX_TOP==the length of indexes[] */
    URES_INDEX_STRINGS_TOP,     /* [1] contains the top of the strings, */
                                /*     same as the bottom of resources, rounded up */
    URES_INDEX_RESOURCES_TOP,   /* [2] contains the top of all resources */
    URES_INDEX_BUNDLE_TOP,      /* [3] contains the top of the bundle, */
                                /*     in case it were ever different from [2] */
    URES_INDEX_MAX_TABLE_LENGTH,/* [4] max. length of any table */
    URES_INDEX_ATTRIBUTES,      /* [5] attributes bit set, see URES_ATT_* (new in formatVersion 1.2) */
    URES_INDEX_TOP
};

/* number of bytes at the beginning of the bundle before the strings start */
enum {
    URES_STRINGS_BOTTOM=(1+URES_INDEX_TOP)*4
};

#define URES_ATT_NO_FALLBACK 1


typedef struct {
    UDataMemory *data;
    Resource *pRoot;
    Resource rootRes;
    UBool noFallback; /* see URES_ATT_NO_FALLBACK */
} ResourceData;

U_CFUNC UBool
res_load(ResourceData *pResData,
         const char *path, const char *name, UErrorCode *errorCode);

U_CFUNC void
res_unload(ResourceData *pResData);

U_CFUNC const UChar *
res_getString(const ResourceData *pResData, const Resource res, int32_t *pLength);

U_CFUNC const UChar *
res_getAlias(const ResourceData *pResData, const Resource res, int32_t *pLength);

U_CFUNC const uint8_t *
res_getBinary(const ResourceData *pResData, const Resource res, int32_t *pLength);

U_CFUNC const int32_t *
res_getIntVector(const ResourceData *pResData, const Resource res, int32_t *pLength);

U_CFUNC Resource
res_getResource(const ResourceData *pResData, const char *key);

U_CFUNC int32_t
res_countArrayItems(const ResourceData *pResData, const Resource res);

U_CFUNC Resource res_getArrayItem(const ResourceData *pResData, Resource array, const int32_t indexS);
U_CFUNC Resource res_getTableItemByIndex(const ResourceData *pResData, Resource table, int32_t indexS, const char ** key);
U_CFUNC Resource res_getTableItemByKey(const ResourceData *pResData, Resource table, int32_t *indexS, const char* * key);

U_CFUNC Resource res_findResource(const ResourceData *pResData, Resource r, char** path, const char** key);

U_CAPI int32_t U_EXPORT2
ures_swap(const UDataSwapper *ds,
          const void *inData, int32_t length, void *outData,
          UErrorCode *pErrorCode);

#endif
