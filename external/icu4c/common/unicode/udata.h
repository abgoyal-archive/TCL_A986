
#ifndef __UDATA_H__
#define __UDATA_H__

#include "unicode/utypes.h"
#include "unicode/localpointer.h"

U_CDECL_BEGIN

 
#ifndef U_HIDE_INTERNAL_API
#define U_TREE_SEPARATOR '-'

#define U_TREE_SEPARATOR_STRING "-"

#define U_TREE_ENTRY_SEP_CHAR '/'

#define U_TREE_ENTRY_SEP_STRING "/"

#define U_ICUDATA_ALIAS "ICUDATA"

#endif /* U_HIDE_INTERNAL_API */

typedef struct {
    /** sizeof(UDataInfo)
     *  @stable ICU 2.0 */
    uint16_t size;

    /** unused, set to 0 
     *  @stable ICU 2.0*/
    uint16_t reservedWord;

    /* platform data properties */
    /** 0 for little-endian machine, 1 for big-endian
     *  @stable ICU 2.0 */
    uint8_t isBigEndian;

    /** see U_CHARSET_FAMILY values in utypes.h 
     *  @stable ICU 2.0*/
    uint8_t charsetFamily;

    /** sizeof(UChar), one of { 1, 2, 4 } 
     *  @stable ICU 2.0*/
    uint8_t sizeofUChar;

    /** unused, set to 0 
     *  @stable ICU 2.0*/
    uint8_t reservedByte;

    /** data format identifier 
     *  @stable ICU 2.0*/
    uint8_t dataFormat[4];

    /** versions: [0] major [1] minor [2] milli [3] micro 
     *  @stable ICU 2.0*/
    uint8_t formatVersion[4];

    /** versions: [0] major [1] minor [2] milli [3] micro 
     *  @stable ICU 2.0*/
    uint8_t dataVersion[4];
} UDataInfo;

/* API for reading data -----------------------------------------------------*/

typedef struct UDataMemory UDataMemory;

typedef UBool U_CALLCONV
UDataMemoryIsAcceptable(void *context,
                        const char *type, const char *name,
                        const UDataInfo *pInfo);


U_STABLE UDataMemory * U_EXPORT2
udata_open(const char *path, const char *type, const char *name,
           UErrorCode *pErrorCode);

U_STABLE UDataMemory * U_EXPORT2
udata_openChoice(const char *path, const char *type, const char *name,
                 UDataMemoryIsAcceptable *isAcceptable, void *context,
                 UErrorCode *pErrorCode);

U_STABLE void U_EXPORT2
udata_close(UDataMemory *pData);

#if U_SHOW_CPLUSPLUS_API

U_NAMESPACE_BEGIN

U_DEFINE_LOCAL_OPEN_POINTER(LocalUDataMemoryPointer, UDataMemory, udata_close);

U_NAMESPACE_END

#endif

U_STABLE const void * U_EXPORT2
udata_getMemory(UDataMemory *pData);

U_STABLE void U_EXPORT2
udata_getInfo(UDataMemory *pData, UDataInfo *pInfo);

U_STABLE void U_EXPORT2
udata_setCommonData(const void *data, UErrorCode *err);


U_STABLE void U_EXPORT2
udata_setAppData(const char *packageName, const void *data, UErrorCode *err);

typedef enum UDataFileAccess {
    /** ICU looks for data in single files first, then in packages. (default) */
    UDATA_FILES_FIRST,
    /** ICU only loads data from packages, not from single files. */
    UDATA_ONLY_PACKAGES,
    /** ICU loads data from packages first, and only from single files
        if the data cannot be found in a package. */
    UDATA_PACKAGES_FIRST,
    /** ICU does not access the file system for data loading. */
    UDATA_NO_FILES,
    /** An alias for the default access mode. */
    UDATA_DEFAULT_ACCESS = UDATA_FILES_FIRST,
    UDATA_FILE_ACCESS_COUNT
} UDataFileAccess;

U_STABLE void U_EXPORT2
udata_setFileAccess(UDataFileAccess access, UErrorCode *status);

U_CDECL_END

#endif
