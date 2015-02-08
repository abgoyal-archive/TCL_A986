
#ifndef URESIMP_H
#define URESIMP_H

#include "unicode/ures.h"

#include "uresdata.h"

#define kRootLocaleName         "root"
#define kPoolBundleName         "pool"


#define kDefaultMinorVersion    "0"
#define kVersionSeparator       "."
#define kVersionTag             "Version"

#define MAGIC1 19700503
#define MAGIC2 19641227

#define URES_MAX_ALIAS_LEVEL 256
#define URES_MAX_BUFFER_SIZE 256


struct UResourceDataEntry;
typedef struct UResourceDataEntry UResourceDataEntry;

struct UResourceDataEntry {
    char *fName; /* name of the locale for bundle - still to decide whether it is original or fallback */
    char *fPath; /* path to bundle - used for distinguishing between resources with the same name */
    UResourceDataEntry *fParent; /*next resource in fallback chain*/
    UResourceDataEntry *fAlias;
    UResourceDataEntry *fPool;
    ResourceData fData; /* data for low level access */
    char fNameBuffer[3]; /* A small buffer of free space for fName. The free space is due to struct padding. */
    uint32_t fCountExisting; /* how much is this resource used */
    UErrorCode fBogus;
    /* int32_t fHashKey;*/ /* for faster access in the hashtable */
};

#define RES_BUFSIZE 64
#define RES_PATH_SEPARATOR   '/'
#define RES_PATH_SEPARATOR_S   "/"

struct UResourceBundle {
    const char *fKey; /*tag*/
    UResourceDataEntry *fData; /*for low-level access*/
    char *fVersion;
    UResourceDataEntry *fTopLevelData; /* for getting the valid locale */
    char *fResPath; /* full path to the resource: "zh_TW/CollationElements/Sequence" */
    ResourceData fResData;
    char fResBuf[RES_BUFSIZE];
    int32_t fResPathLen;
    Resource fRes;
    UBool fHasFallback;
    UBool fIsTopLevel;
    uint32_t fMagic1;   /* For determining if it's a stack object */
    uint32_t fMagic2;   /* For determining if it's a stack object */
    int32_t fIndex;
    int32_t fSize;

    /*const UResourceBundle *fParentRes;*/ /* needed to get the actual locale for a child resource */
};

U_CAPI void U_EXPORT2 ures_initStackObject(UResourceBundle* resB);

/* Some getters used by the copy constructor */
U_CFUNC const char* ures_getName(const UResourceBundle* resB);
#ifdef URES_DEBUG
U_CFUNC const char* ures_getPath(const UResourceBundle* resB);
U_CAPI UBool U_EXPORT2 ures_dumpCacheContents(void);
#endif
/*U_CFUNC void ures_appendResPath(UResourceBundle *resB, const char* toAdd, int32_t lenToAdd);*/
/*U_CFUNC void ures_setResPath(UResourceBundle *resB, const char* toAdd);*/
/*U_CFUNC void ures_freeResPath(UResourceBundle *resB);*/

/* Candidates for export */
U_CFUNC UResourceBundle *ures_copyResb(UResourceBundle *r, const UResourceBundle *original, UErrorCode *status);

U_CAPI UResourceBundle* U_EXPORT2
ures_findResource(const char* pathToResource, 
                  UResourceBundle *fillIn, UErrorCode *status); 

U_CAPI UResourceBundle* U_EXPORT2
ures_findSubResource(const UResourceBundle *resB, 
                     char* pathToResource, 
                     UResourceBundle *fillIn, UErrorCode *status);

U_INTERNAL int32_t U_EXPORT2
ures_getFunctionalEquivalent(char *result, int32_t resultCapacity, 
                             const char *path, const char *resName, const char *keyword, const char *locid,
                             UBool *isAvailable, UBool omitDefault, UErrorCode *status);

U_INTERNAL UEnumeration* U_EXPORT2
ures_getKeywordValues(const char *path, const char *keyword, UErrorCode *status);


U_INTERNAL UResourceBundle* U_EXPORT2 
ures_getByKeyWithFallback(const UResourceBundle *resB, 
                          const char* inKey, 
                          UResourceBundle *fillIn, 
                          UErrorCode *status);


U_INTERNAL const UChar* U_EXPORT2 
ures_getStringByKeyWithFallback(const UResourceBundle *resB, 
                          const char* inKey,  
                          int32_t* len,
                          UErrorCode *status);

U_INTERNAL void U_EXPORT2
ures_getVersionByKey(const UResourceBundle *resB,
                     const char *key,
                     UVersionInfo ver,
                     UErrorCode *status);


#endif /*URESIMP_H*/
