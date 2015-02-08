

#ifndef UCOL_H
#define UCOL_H

#include "unicode/utypes.h"

#if !UCONFIG_NO_COLLATION

#include "unicode/unorm.h"
#include "unicode/parseerr.h"
#include "unicode/uloc.h"
#include "unicode/uset.h"


struct collIterate;
typedef struct collIterate collIterate;

struct UCollator;
typedef struct UCollator UCollator;


typedef enum {
  /** string a == string b */
  UCOL_EQUAL    = 0,
  /** string a > string b */
  UCOL_GREATER    = 1,
  /** string a < string b */
  UCOL_LESS    = -1
} UCollationResult ;


typedef enum {
  /** accepted by most attributes */
  UCOL_DEFAULT = -1,

  /** Primary collation strength */
  UCOL_PRIMARY = 0,
  /** Secondary collation strength */
  UCOL_SECONDARY = 1,
  /** Tertiary collation strength */
  UCOL_TERTIARY = 2,
  /** Default collation strength */
  UCOL_DEFAULT_STRENGTH = UCOL_TERTIARY,
  UCOL_CE_STRENGTH_LIMIT,
  /** Quaternary collation strength */
  UCOL_QUATERNARY=3,
  /** Identical collation strength */
  UCOL_IDENTICAL=15,
  UCOL_STRENGTH_LIMIT,

  /** Turn the feature off - works for UCOL_FRENCH_COLLATION, 
      UCOL_CASE_LEVEL, UCOL_HIRAGANA_QUATERNARY_MODE
      & UCOL_DECOMPOSITION_MODE*/
  UCOL_OFF = 16,
  /** Turn the feature on - works for UCOL_FRENCH_COLLATION, 
      UCOL_CASE_LEVEL, UCOL_HIRAGANA_QUATERNARY_MODE
      & UCOL_DECOMPOSITION_MODE*/
  UCOL_ON = 17,
  
  /** Valid for UCOL_ALTERNATE_HANDLING. Alternate handling will be shifted */
  UCOL_SHIFTED = 20,
  /** Valid for UCOL_ALTERNATE_HANDLING. Alternate handling will be non ignorable */
  UCOL_NON_IGNORABLE = 21,

  /** Valid for UCOL_CASE_FIRST - 
      lower case sorts before upper case */
  UCOL_LOWER_FIRST = 24,
  /** upper case sorts before lower case */
  UCOL_UPPER_FIRST = 25,

  UCOL_ATTRIBUTE_VALUE_COUNT

} UColAttributeValue;

typedef UColAttributeValue UCollationStrength;

typedef enum {
     /** Attribute for direction of secondary weights - used in French.\ 
      * Acceptable values are UCOL_ON, which results in secondary weights
      * being considered backwards and UCOL_OFF which treats secondary
      * weights in the order they appear.*/
     UCOL_FRENCH_COLLATION, 
     /** Attribute for handling variable elements.\ 
      * Acceptable values are UCOL_NON_IGNORABLE (default)
      * which treats all the codepoints with non-ignorable 
      * primary weights in the same way,
      * and UCOL_SHIFTED which causes codepoints with primary 
      * weights that are equal or below the variable top value
      * to be ignored on primary level and moved to the quaternary 
      * level.*/
     UCOL_ALTERNATE_HANDLING, 
     /** Controls the ordering of upper and lower case letters.\ 
      * Acceptable values are UCOL_OFF (default), which orders
      * upper and lower case letters in accordance to their tertiary
      * weights, UCOL_UPPER_FIRST which forces upper case letters to 
      * sort before lower case letters, and UCOL_LOWER_FIRST which does 
      * the opposite. */
     UCOL_CASE_FIRST, 
     /** Controls whether an extra case level (positioned before the third
      * level) is generated or not.\ Acceptable values are UCOL_OFF (default), 
      * when case level is not generated, and UCOL_ON which causes the case
      * level to be generated.\ Contents of the case level are affected by
      * the value of UCOL_CASE_FIRST attribute.\ A simple way to ignore 
      * accent differences in a string is to set the strength to UCOL_PRIMARY
      * and enable case level. */
     UCOL_CASE_LEVEL,
     /** Controls whether the normalization check and necessary normalizations
      * are performed.\ When set to UCOL_OFF (default) no normalization check
      * is performed.\ The correctness of the result is guaranteed only if the 
      * input data is in so-called FCD form (see users manual for more info).\ 
      * When set to UCOL_ON, an incremental check is performed to see whether the input data
      * is in the FCD form.\ If the data is not in the FCD form, incremental 
      * NFD normalization is performed. */
     UCOL_NORMALIZATION_MODE, 
     /** An alias for UCOL_NORMALIZATION_MODE attribute */
     UCOL_DECOMPOSITION_MODE = UCOL_NORMALIZATION_MODE,
     /** The strength attribute.\ Can be either UCOL_PRIMARY, UCOL_SECONDARY,
      * UCOL_TERTIARY, UCOL_QUATERNARY or UCOL_IDENTICAL.\ The usual strength
      * for most locales (except Japanese) is tertiary.\ Quaternary strength 
      * is useful when combined with shifted setting for alternate handling
      * attribute and for JIS x 4061 collation, when it is used to distinguish
      * between Katakana  and Hiragana (this is achieved by setting the 
      * UCOL_HIRAGANA_QUATERNARY mode to on.\ Otherwise, quaternary level
      * is affected only by the number of non ignorable code points in
      * the string.\ Identical strength is rarely useful, as it amounts 
      * to codepoints of the NFD form of the string. */
     UCOL_STRENGTH,  
     /** when turned on, this attribute 
      * positions Hiragana before all  
      * non-ignorables on quaternary level
      * This is a sneaky way to produce JIS
      * sort order */     
     UCOL_HIRAGANA_QUATERNARY_MODE,
     /** when turned on, this attribute 
      * generates a collation key
      * for the numeric value of substrings
      * of digits. This is a way to get '100' 
      * to sort AFTER '2'.*/          
     UCOL_NUMERIC_COLLATION, 
     UCOL_ATTRIBUTE_COUNT
} UColAttribute;

typedef enum {
  /** Retrieve tailoring only */
  UCOL_TAILORING_ONLY, 
  /** Retrieve UCA rules and tailoring */
  UCOL_FULL_RULES 
} UColRuleOption ;

U_STABLE UCollator* U_EXPORT2 
ucol_open(const char *loc, UErrorCode *status);

U_STABLE UCollator* U_EXPORT2 
ucol_openRules( const UChar        *rules,
                int32_t            rulesLength,
                UColAttributeValue normalizationMode,
                UCollationStrength strength,
                UParseError        *parseError,
                UErrorCode         *status);

U_CAPI UCollator* U_EXPORT2
ucol_openFromShortString( const char *definition,
                          UBool forceDefaults,
                          UParseError *parseError,
                          UErrorCode *status);

U_CAPI int32_t U_EXPORT2
ucol_getContractions( const UCollator *coll,
                  USet *conts,
                  UErrorCode *status);


U_STABLE void U_EXPORT2 
ucol_close(UCollator *coll);

U_STABLE UCollationResult U_EXPORT2 
ucol_strcoll(    const    UCollator    *coll,
        const    UChar        *source,
        int32_t            sourceLength,
        const    UChar        *target,
        int32_t            targetLength);

U_STABLE UBool U_EXPORT2 
ucol_greater(const UCollator *coll,
             const UChar     *source, int32_t sourceLength,
             const UChar     *target, int32_t targetLength);

U_STABLE UBool U_EXPORT2 
ucol_greaterOrEqual(const UCollator *coll,
                    const UChar     *source, int32_t sourceLength,
                    const UChar     *target, int32_t targetLength);

U_STABLE UBool U_EXPORT2 
ucol_equal(const UCollator *coll,
           const UChar     *source, int32_t sourceLength,
           const UChar     *target, int32_t targetLength);

U_STABLE UCollationResult U_EXPORT2 
ucol_strcollIter(  const    UCollator    *coll,
                  UCharIterator *sIter,
                  UCharIterator *tIter,
                  UErrorCode *status);

U_STABLE UCollationStrength U_EXPORT2 
ucol_getStrength(const UCollator *coll);

U_STABLE void U_EXPORT2 
ucol_setStrength(UCollator *coll,
                 UCollationStrength strength);

U_STABLE int32_t U_EXPORT2 
ucol_getDisplayName(    const    char        *objLoc,
            const    char        *dispLoc,
            UChar             *result,
            int32_t         resultLength,
            UErrorCode        *status);

U_STABLE const char* U_EXPORT2 
ucol_getAvailable(int32_t index);

U_STABLE int32_t U_EXPORT2 
ucol_countAvailable(void);

#if !UCONFIG_NO_SERVICE
U_DRAFT UEnumeration* U_EXPORT2
ucol_openAvailableLocales(UErrorCode *status);
#endif

U_DRAFT UEnumeration* U_EXPORT2
ucol_getKeywords(UErrorCode *status);

U_DRAFT UEnumeration* U_EXPORT2
ucol_getKeywordValues(const char *keyword, UErrorCode *status);

U_DRAFT int32_t U_EXPORT2
ucol_getFunctionalEquivalent(char* result, int32_t resultCapacity,
                             const char* keyword, const char* locale,
                             UBool* isAvailable, UErrorCode* status);

U_STABLE const UChar* U_EXPORT2 
ucol_getRules(    const    UCollator    *coll, 
        int32_t            *length);

U_CAPI int32_t U_EXPORT2
ucol_getShortDefinitionString(const UCollator *coll,
                              const char *locale,
                              char *buffer,
                              int32_t capacity,
                              UErrorCode *status);


U_CAPI int32_t U_EXPORT2
ucol_normalizeShortDefinitionString(const char *source,
                                    char *destination,
                                    int32_t capacity,
                                    UParseError *parseError,
                                    UErrorCode *status);
        

U_STABLE int32_t U_EXPORT2 
ucol_getSortKey(const    UCollator    *coll,
        const    UChar        *source,
        int32_t        sourceLength,
        uint8_t        *result,
        int32_t        resultLength);


U_STABLE int32_t U_EXPORT2 
ucol_nextSortKeyPart(const UCollator *coll,
                     UCharIterator *iter,
                     uint32_t state[2],
                     uint8_t *dest, int32_t count,
                     UErrorCode *status);

typedef enum {
  /** lower bound */
  UCOL_BOUND_LOWER = 0,
  /** upper bound that will match strings of exact size */
  UCOL_BOUND_UPPER = 1,
  /** upper bound that will match all the strings that have the same initial substring as the given string */
  UCOL_BOUND_UPPER_LONG = 2,
  UCOL_BOUND_VALUE_COUNT
} UColBoundMode;

U_STABLE int32_t U_EXPORT2 
ucol_getBound(const uint8_t       *source,
        int32_t             sourceLength,
        UColBoundMode       boundType,
        uint32_t            noOfLevels,
        uint8_t             *result,
        int32_t             resultLength,
        UErrorCode          *status);
        
U_STABLE void U_EXPORT2
ucol_getVersion(const UCollator* coll, UVersionInfo info);

U_DRAFT void U_EXPORT2
ucol_getUCAVersion(const UCollator* coll, UVersionInfo info);

U_STABLE int32_t U_EXPORT2 
ucol_mergeSortkeys(const uint8_t *src1, int32_t src1Length,
                   const uint8_t *src2, int32_t src2Length,
                   uint8_t *dest, int32_t destCapacity);

U_STABLE void U_EXPORT2 
ucol_setAttribute(UCollator *coll, UColAttribute attr, UColAttributeValue value, UErrorCode *status);

U_STABLE UColAttributeValue  U_EXPORT2 
ucol_getAttribute(const UCollator *coll, UColAttribute attr, UErrorCode *status);

U_STABLE uint32_t U_EXPORT2 
ucol_setVariableTop(UCollator *coll, 
                    const UChar *varTop, int32_t len, 
                    UErrorCode *status);

U_STABLE uint32_t U_EXPORT2 ucol_getVariableTop(const UCollator *coll, UErrorCode *status);

U_STABLE void U_EXPORT2 
ucol_restoreVariableTop(UCollator *coll, const uint32_t varTop, UErrorCode *status);

U_STABLE UCollator* U_EXPORT2 
ucol_safeClone(const UCollator *coll,
               void            *stackBuffer,
               int32_t         *pBufferSize,
               UErrorCode      *status);

#define U_COL_SAFECLONE_BUFFERSIZE 512

U_STABLE int32_t U_EXPORT2 
ucol_getRulesEx(const UCollator *coll, UColRuleOption delta, UChar *buffer, int32_t bufferLen);

U_DEPRECATED const char * U_EXPORT2
ucol_getLocale(const UCollator *coll, ULocDataLocaleType type, UErrorCode *status);


U_DRAFT const char * U_EXPORT2
ucol_getLocaleByType(const UCollator *coll, ULocDataLocaleType type, UErrorCode *status);

U_STABLE USet * U_EXPORT2
ucol_getTailoredSet(const UCollator *coll, UErrorCode *status);

#define UCOL_SIT_COLLATOR_NOT_ENCODABLE 0x80000000

U_INTERNAL uint32_t U_EXPORT2
ucol_collatorToIdentifier(const UCollator *coll,
                          const char *locale,
                          UErrorCode *status);

U_INTERNAL UCollator* U_EXPORT2
ucol_openFromIdentifier(uint32_t identifier,
                        UBool forceDefaults,
                        UErrorCode *status);


U_INTERNAL int32_t U_EXPORT2
ucol_identifierToShortString(uint32_t identifier,
                             char *buffer,
                             int32_t capacity,
                             UBool forceDefaults,
                             UErrorCode *status);

U_INTERNAL uint32_t U_EXPORT2
ucol_shortStringToIdentifier(const char *definition,
                             UBool forceDefaults,
                             UErrorCode *status);



U_INTERNAL UColAttributeValue  U_EXPORT2
ucol_getAttributeOrDefault(const UCollator *coll, UColAttribute attr, UErrorCode *status);

U_INTERNAL UBool U_EXPORT2
ucol_equals(const UCollator *source, const UCollator *target);

U_INTERNAL int32_t U_EXPORT2
ucol_getUnsafeSet( const UCollator *coll,
                  USet *unsafe,
                  UErrorCode *status);

U_INTERNAL void U_EXPORT2
ucol_forgetUCA(void);

U_INTERNAL void U_EXPORT2
ucol_prepareShortStringOpen( const char *definition,
                          UBool forceDefaults,
                          UParseError *parseError,
                          UErrorCode *status);

U_DRAFT int32_t U_EXPORT2
ucol_cloneBinary(const UCollator *coll,
                 uint8_t *buffer, int32_t capacity,
                 UErrorCode *status);

U_DRAFT UCollator* U_EXPORT2
ucol_openBinary(const uint8_t *bin, int32_t length, 
                const UCollator *base, 
                UErrorCode *status);


#endif /* #if !UCONFIG_NO_COLLATION */

#endif

