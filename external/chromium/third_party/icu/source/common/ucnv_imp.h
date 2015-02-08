
#ifndef UCNV_IMP_H
#define UCNV_IMP_H

#include "unicode/utypes.h"

#if !UCONFIG_NO_CONVERSION

#include "unicode/uloc.h"
#include "ucnv_bld.h"

#define UCNV_FAST_IS_UTF8(name) \
    (((name[0]=='U' ? \
      (                name[1]=='T' && name[2]=='F') : \
      (name[0]=='u' && name[1]=='t' && name[2]=='f'))) \
  && (name[3]=='-' ? \
     (name[4]=='8' && name[5]==0) : \
     (name[3]=='8' && name[4]==0)))

typedef struct {
    char cnvName[UCNV_MAX_CONVERTER_NAME_LENGTH];
    char locale[ULOC_FULLNAME_CAPACITY];
    uint32_t options;
} UConverterNamePieces;

U_CFUNC UBool
ucnv_canCreateConverter(const char *converterName, UErrorCode *err);

UConverter *ucnv_createConverter (UConverter *myUConverter, const char *converterName, UErrorCode * err);

U_CFUNC UConverter *
ucnv_createAlgorithmicConverter(UConverter *myUConverter,
                                UConverterType type,
                                const char *locale, uint32_t options,
                                UErrorCode *err);

UConverter*
ucnv_createConverterFromSharedData(UConverter *myUConverter,
                                   UConverterSharedData *mySharedConverterData,
                                   UConverterLoadArgs *pArgs,
                                   UErrorCode *err);

UConverter* ucnv_createConverterFromPackage(const char *packageName, const char *converterName,  
                                            UErrorCode *err);

UConverterSharedData *
ucnv_loadSharedData(const char *converterName,
                    UConverterNamePieces *pieces,
                    UConverterLoadArgs *pArgs,
                    UErrorCode * err);

void
ucnv_unloadSharedDataIfReady(UConverterSharedData *sharedData);

void
ucnv_incrementRefCount(UConverterSharedData *sharedData);

#define UCNV_TO_U_DEFAULT_CALLBACK ((UConverterToUCallback) UCNV_TO_U_CALLBACK_SUBSTITUTE)
#define UCNV_FROM_U_DEFAULT_CALLBACK ((UConverterFromUCallback) UCNV_FROM_U_CALLBACK_SUBSTITUTE)

#endif

#endif /* _UCNV_IMP */
