
#ifndef __UNEWDATA_H__
#define __UNEWDATA_H__

#include "unicode/utypes.h"
#include "unicode/udata.h"

/* API for writing data -----------------------------------------------------*/

/** @memo Forward declaration of the data memory creation type. */
typedef struct UNewDataMemory UNewDataMemory;

U_CAPI UNewDataMemory * U_EXPORT2
udata_create(const char *dir, const char *type, const char *name,
             const UDataInfo *pInfo,
             const char *comment,
             UErrorCode *pErrorCode);

/** @memo Close a newly written binary file. */
U_CAPI uint32_t U_EXPORT2
udata_finish(UNewDataMemory *pData, UErrorCode *pErrorCode);

/** @memo Write an 8-bit byte to the file. */
U_CAPI void U_EXPORT2
udata_write8(UNewDataMemory *pData, uint8_t byte);

/** @memo Write a 16-bit word to the file. */
U_CAPI void U_EXPORT2
udata_write16(UNewDataMemory *pData, uint16_t word);

/** @memo Write a 32-bit word to the file. */
U_CAPI void U_EXPORT2
udata_write32(UNewDataMemory *pData, uint32_t wyde);

/** @memo Write a block of bytes to the file. */
U_CAPI void U_EXPORT2
udata_writeBlock(UNewDataMemory *pData, const void *s, int32_t length);

/** @memo Write a block of arbitrary padding bytes to the file. */
U_CAPI void U_EXPORT2
udata_writePadding(UNewDataMemory *pData, int32_t length);

/** @memo Write a <code>char*</code> string of platform "invariant characters" to the file. */
U_CAPI void U_EXPORT2
udata_writeString(UNewDataMemory *pData, const char *s, int32_t length);

/** @memo Write a <code>UChar*</code> string of Unicode character code units to the file. */
U_CAPI void U_EXPORT2
udata_writeUString(UNewDataMemory *pData, const UChar *s, int32_t length);



#endif
