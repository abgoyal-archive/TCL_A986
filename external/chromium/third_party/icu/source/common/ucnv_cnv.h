
#ifndef UCNV_CNV_H
#define UCNV_CNV_H

#include "unicode/utypes.h"

#if !UCONFIG_NO_CONVERSION

#include "unicode/ucnv.h"
#include "unicode/ucnv_err.h"
#include "unicode/uset.h"
#include "uset_imp.h"

U_CDECL_BEGIN

/* this is used in fromUnicode DBCS tables as an "unassigned" marker */
#define missingCharMarker 0xFFFF


/** Forward declaration, see ucnv_bld.h */
struct UConverterSharedData;
typedef struct UConverterSharedData UConverterSharedData;

/* function types for UConverterImpl ---------------------------------------- */

/* struct with arguments for UConverterLoad and ucnv_load() */
typedef struct {
    int32_t size;               /* sizeof(UConverterLoadArgs) */
    int32_t nestedLoads;        /* count nested ucnv_load() calls */
    UBool onlyTestIsLoadable;   /* input: don't actually load */
    UBool reserved0;            /* reserved - for good alignment of the pointers */
    int16_t reserved;           /* reserved - for good alignment of the pointers */
    uint32_t options;
    const char *pkg, *name, *locale;
} UConverterLoadArgs;

typedef void (*UConverterLoad) (UConverterSharedData *sharedData,
                                UConverterLoadArgs *pArgs,
                                const uint8_t *raw, UErrorCode *pErrorCode);
typedef void (*UConverterUnload) (UConverterSharedData *sharedData);

typedef void (*UConverterOpen) (UConverter *cnv, UConverterLoadArgs *pArgs, UErrorCode *pErrorCode);
typedef void (*UConverterClose) (UConverter *cnv);

typedef enum UConverterResetChoice {
    UCNV_RESET_BOTH,
    UCNV_RESET_TO_UNICODE,
    UCNV_RESET_FROM_UNICODE
} UConverterResetChoice;

typedef void (*UConverterReset) (UConverter *cnv, UConverterResetChoice choice);

typedef void (*UConverterToUnicode) (UConverterToUnicodeArgs *, UErrorCode *);

typedef void (*UConverterFromUnicode) (UConverterFromUnicodeArgs *, UErrorCode *);

typedef void (*UConverterConvert) (UConverterFromUnicodeArgs *pFromUArgs,
                                   UConverterToUnicodeArgs *pToUArgs,
                                   UErrorCode *pErrorCode);

typedef UChar32 (*UConverterGetNextUChar) (UConverterToUnicodeArgs *, UErrorCode *);

typedef void (*UConverterGetStarters)(const UConverter* converter,
                                      UBool starters[256],
                                      UErrorCode *pErrorCode);

typedef const char * (*UConverterGetName) (const UConverter *cnv);

typedef void (*UConverterWriteSub) (UConverterFromUnicodeArgs *pArgs, int32_t offsetIndex, UErrorCode *pErrorCode);

typedef UConverter * (*UConverterSafeClone) (const UConverter   *cnv, 
                                             void               *stackBuffer,
                                             int32_t            *pBufferSize, 
                                             UErrorCode         *status);

typedef enum UConverterSetFilter {
    UCNV_SET_FILTER_NONE,
    UCNV_SET_FILTER_DBCS_ONLY,
    UCNV_SET_FILTER_2022_CN,
    UCNV_SET_FILTER_SJIS,
    UCNV_SET_FILTER_GR94DBCS,
    UCNV_SET_FILTER_HZ,
    UCNV_SET_FILTER_COUNT
} UConverterSetFilter;

typedef void (*UConverterGetUnicodeSet) (const UConverter *cnv,
                                         const USetAdder *sa,
                                         UConverterUnicodeSet which,
                                         UErrorCode *pErrorCode);

UBool CONVERSION_U_SUCCESS (UErrorCode err);

struct UConverterImpl {
    UConverterType type;

    UConverterLoad load;
    UConverterUnload unload;

    UConverterOpen open;
    UConverterClose close;
    UConverterReset reset;

    UConverterToUnicode toUnicode;
    UConverterToUnicode toUnicodeWithOffsets;
    UConverterFromUnicode fromUnicode;
    UConverterFromUnicode fromUnicodeWithOffsets;
    UConverterGetNextUChar getNextUChar;

    UConverterGetStarters getStarters;
    UConverterGetName getName;
    UConverterWriteSub writeSub;
    UConverterSafeClone safeClone;
    UConverterGetUnicodeSet getUnicodeSet;

    UConverterConvert toUTF8;
    UConverterConvert fromUTF8;
};

extern const UConverterSharedData
    _MBCSData, _Latin1Data,
    _UTF8Data, _UTF16BEData, _UTF16LEData, _UTF32BEData, _UTF32LEData,
    _ISO2022Data, 
    _LMBCSData1,_LMBCSData2, _LMBCSData3, _LMBCSData4, _LMBCSData5, _LMBCSData6,
    _LMBCSData8,_LMBCSData11,_LMBCSData16,_LMBCSData17,_LMBCSData18,_LMBCSData19,
    _HZData,_ISCIIData, _SCSUData, _ASCIIData,
    _UTF7Data, _Bocu1Data, _UTF16Data, _UTF32Data, _CESU8Data, _IMAPData;

U_CDECL_END

/** Always use fallbacks from codepage to Unicode */
#define TO_U_USE_FALLBACK(useFallback) TRUE
#define UCNV_TO_U_USE_FALLBACK(cnv) TRUE

/** Use fallbacks from Unicode to codepage when cnv->useFallback or for private-use code points */
#define IS_PRIVATE_USE(c) ((uint32_t)((c)-0xe000)<0x1900 || (uint32_t)((c)-0xf0000)<0x20000)
#define FROM_U_USE_FALLBACK(useFallback, c) ((useFallback) || IS_PRIVATE_USE(c))
#define UCNV_FROM_U_USE_FALLBACK(cnv, c) FROM_U_USE_FALLBACK((cnv)->useFallback, c)

#define UCNV_GET_NEXT_UCHAR_USE_TO_U -9

U_CFUNC void
ucnv_getCompleteUnicodeSet(const UConverter *cnv,
                   const USetAdder *sa,
                   UConverterUnicodeSet which,
                   UErrorCode *pErrorCode);

U_CFUNC void
ucnv_getNonSurrogateUnicodeSet(const UConverter *cnv,
                               const USetAdder *sa,
                               UConverterUnicodeSet which,
                               UErrorCode *pErrorCode);

U_CFUNC void
ucnv_fromUWriteBytes(UConverter *cnv,
                     const char *bytes, int32_t length,
                     char **target, const char *targetLimit,
                     int32_t **offsets,
                     int32_t sourceIndex,
                     UErrorCode *pErrorCode);
U_CFUNC void
ucnv_toUWriteUChars(UConverter *cnv,
                    const UChar *uchars, int32_t length,
                    UChar **target, const UChar *targetLimit,
                    int32_t **offsets,
                    int32_t sourceIndex,
                    UErrorCode *pErrorCode);

U_CFUNC void
ucnv_toUWriteCodePoint(UConverter *cnv,
                       UChar32 c,
                       UChar **target, const UChar *targetLimit,
                       int32_t **offsets,
                       int32_t sourceIndex,
                       UErrorCode *pErrorCode);

#endif

#endif /* UCNV_CNV */
