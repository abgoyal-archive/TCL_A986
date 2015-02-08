
#ifndef PRSCMNTS_H
#define PRSCMNTS_H 1

#if UCONFIG_NO_REGULAR_EXPRESSIONS==0 /* donot compile when no RegularExpressions are available */

enum UParseCommentsOption {
    UPC_TRANSLATE,
    UPC_NOTE,
    UPC_LIMIT
};

typedef enum UParseCommentsOption UParseCommentsOption;

U_CFUNC int32_t 
getNote(const UChar* source, int32_t srcLen,
        UChar** dest, int32_t destCapacity,
        UErrorCode* status);
U_CFUNC int32_t 
removeCmtText(UChar* source, int32_t srcLen, UErrorCode* status);

U_CFUNC int32_t
getDescription( const UChar* source, int32_t srcLen,
                UChar** dest, int32_t destCapacity,
                UErrorCode* status);
U_CFUNC int32_t
getTranslate( const UChar* source, int32_t srcLen,
              UChar** dest, int32_t destCapacity,
              UErrorCode* status);

U_CFUNC int32_t
getAt(const UChar* source, int32_t srcLen,
        UChar** dest, int32_t destCapacity,
        int32_t index,
        UParseCommentsOption option,
        UErrorCode* status);

U_CFUNC int32_t
getCount(const UChar* source, int32_t srcLen, 
         UParseCommentsOption option, UErrorCode *status);

#endif /* UCONFIG_NO_REGULAR_EXPRESSIONS */

#endif

