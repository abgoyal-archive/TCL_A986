
#ifndef __UDATASWP_H__
#define __UDATASWP_H__

#include <stdarg.h>
#include "unicode/utypes.h"

/* forward declaration */

U_CDECL_BEGIN

struct UDataSwapper;
typedef struct UDataSwapper UDataSwapper;

typedef int32_t U_CALLCONV
UDataSwapFn(const UDataSwapper *ds,
            const void *inData, int32_t length, void *outData,
            UErrorCode *pErrorCode);

typedef uint16_t U_CALLCONV
UDataReadUInt16(uint16_t x);

typedef uint32_t U_CALLCONV
UDataReadUInt32(uint32_t x);

typedef void U_CALLCONV
UDataWriteUInt16(uint16_t *p, uint16_t x);

typedef void U_CALLCONV
UDataWriteUInt32(uint32_t *p, uint32_t x);

typedef int32_t U_CALLCONV
UDataCompareInvChars(const UDataSwapper *ds,
                     const char *outString, int32_t outLength,
                     const UChar *localString, int32_t localLength);

typedef void U_CALLCONV
UDataPrintError(void *context, const char *fmt, va_list args);

struct UDataSwapper {
    /** Input endianness. @internal ICU 2.8 */
    UBool inIsBigEndian;
    /** Input charset family. @see U_CHARSET_FAMILY @internal ICU 2.8 */
    uint8_t inCharset;
    /** Output endianness. @internal ICU 2.8 */
    UBool outIsBigEndian;
    /** Output charset family. @see U_CHARSET_FAMILY @internal ICU 2.8 */
    uint8_t outCharset;

    /* basic functions for reading data values */

    /** Convert one uint16_t from input to platform endianness. @internal ICU 2.8 */
    UDataReadUInt16 *readUInt16;
    /** Convert one uint32_t from input to platform endianness. @internal ICU 2.8 */
    UDataReadUInt32 *readUInt32;
    /** Compare an invariant-character output string with a local one. @internal ICU 2.8 */
    UDataCompareInvChars *compareInvChars;

    /* basic functions for writing data values */

    /** Convert one uint16_t from platform to input endianness. @internal ICU 2.8 */
    UDataWriteUInt16 *writeUInt16;
    /** Convert one uint32_t from platform to input endianness. @internal ICU 2.8 */
    UDataWriteUInt32 *writeUInt32;

    /* basic functions for data transformations */

    /** Transform an array of 16-bit integers. @internal ICU 2.8 */
    UDataSwapFn *swapArray16;
    /** Transform an array of 32-bit integers. @internal ICU 2.8 */
    UDataSwapFn *swapArray32;
    /** Transform an invariant-character string. @internal ICU 2.8 */
    UDataSwapFn *swapInvChars;

    /**
     * Function for message output when an error occurs during data swapping.
     * Can be NULL.
     * @internal ICU 2.8
     */
    UDataPrintError *printError;
    /** Context pointer for printError. @internal ICU 2.8 */
    void *printErrorContext;
};

U_CDECL_END

U_CAPI UDataSwapper * U_EXPORT2
udata_openSwapper(UBool inIsBigEndian, uint8_t inCharset,
                  UBool outIsBigEndian, uint8_t outCharset,
                  UErrorCode *pErrorCode);

U_CAPI UDataSwapper * U_EXPORT2
udata_openSwapperForInputData(const void *data, int32_t length,
                              UBool outIsBigEndian, uint8_t outCharset,
                              UErrorCode *pErrorCode);

U_CAPI void U_EXPORT2
udata_closeSwapper(UDataSwapper *ds);

U_CAPI int32_t U_EXPORT2
udata_swapDataHeader(const UDataSwapper *ds,
                     const void *inData, int32_t length, void *outData,
                     UErrorCode *pErrorCode);

U_CAPI int16_t U_EXPORT2
udata_readInt16(const UDataSwapper *ds, int16_t x);

U_CAPI int32_t U_EXPORT2
udata_readInt32(const UDataSwapper *ds, int32_t x);

U_CAPI int32_t U_EXPORT2
udata_swapInvStringBlock(const UDataSwapper *ds,
                         const void *inData, int32_t length, void *outData,
                         UErrorCode *pErrorCode);

U_CAPI void U_EXPORT2
udata_printError(const UDataSwapper *ds,
                 const char *fmt,
                 ...);

/* internal exports from putil.c -------------------------------------------- */

/* declared here to keep them out of the public putil.h */

U_CAPI int32_t U_EXPORT2
uprv_ebcdicFromAscii(const UDataSwapper *ds,
                     const void *inData, int32_t length, void *outData,
                     UErrorCode *pErrorCode);

U_CFUNC int32_t
uprv_copyAscii(const UDataSwapper *ds,
               const void *inData, int32_t length, void *outData,
               UErrorCode *pErrorCode);

U_CFUNC int32_t
uprv_asciiFromEbcdic(const UDataSwapper *ds,
                     const void *inData, int32_t length, void *outData,
                     UErrorCode *pErrorCode);

U_CFUNC int32_t
uprv_copyEbcdic(const UDataSwapper *ds,
                const void *inData, int32_t length, void *outData,
                UErrorCode *pErrorCode);

U_CFUNC int32_t
uprv_compareInvAscii(const UDataSwapper *ds,
                     const char *outString, int32_t outLength,
                     const UChar *localString, int32_t localLength);

U_CFUNC int32_t
uprv_compareInvEbcdic(const UDataSwapper *ds,
                      const char *outString, int32_t outLength,
                      const UChar *localString, int32_t localLength);

/* material... -------------------------------------------------------------- */

#if 0

/* udata.h */

U_CAPI UDataMemory * U_EXPORT2
udata_openSwap(const char *path, const char *type, const char *name,
               UDataMemoryIsAcceptable *isAcceptable, void *isAcceptableContext,
               UDataSwapFn *swap,
               UDataPrintError *printError, void *printErrorContext,
               UErrorCode *pErrorCode);

#endif

#endif
