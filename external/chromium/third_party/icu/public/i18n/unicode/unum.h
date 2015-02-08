
#ifndef _UNUM
#define _UNUM

#include "unicode/utypes.h"

#if !UCONFIG_NO_FORMATTING

#include "unicode/uloc.h"
#include "unicode/umisc.h"
#include "unicode/parseerr.h"

typedef void* UNumberFormat;

typedef enum UNumberFormatStyle {
    /**
     * Decimal format defined by pattern 
     * @stable ICU 3.0
     */
    UNUM_PATTERN_DECIMAL=0,
    /** Decimal format */
    UNUM_DECIMAL=1,
    /** Currency format */
    UNUM_CURRENCY,
    /** Percent format */
    UNUM_PERCENT,
    /** Scientific format */
    UNUM_SCIENTIFIC,
    /** Spellout rule-based format */
    UNUM_SPELLOUT,
    /** 
     * Ordinal rule-based format 
     * @stable ICU 3.0
     */
    UNUM_ORDINAL,
    /** 
     * Duration rule-based format 
     * @stable ICU 3.0
     */
    UNUM_DURATION,
    /** 
     * Numbering system rule-based format 
     * @draft ICU 4.2
     */
    UNUM_NUMBERING_SYSTEM,
    /** 
     * Rule-based format defined by pattern 
     * @stable ICU 3.0
     */
    UNUM_PATTERN_RULEBASED,
    /** Default format */
    UNUM_DEFAULT = UNUM_DECIMAL,
    /** (Alias for UNUM_PATTERN_DECIMAL) */
    UNUM_IGNORE = UNUM_PATTERN_DECIMAL
} UNumberFormatStyle;

typedef enum UNumberFormatRoundingMode {
    UNUM_ROUND_CEILING,
    UNUM_ROUND_FLOOR,
    UNUM_ROUND_DOWN,
    UNUM_ROUND_UP,
    /**
     * Half-even rounding, misspelled name
     * @deprecated, ICU 3.8
     */
    UNUM_FOUND_HALFEVEN,
    UNUM_ROUND_HALFDOWN,
    UNUM_ROUND_HALFUP,
    /**
     * Half-even rounding
     * @stable, ICU 3.8
     */
    UNUM_ROUND_HALFEVEN = UNUM_FOUND_HALFEVEN
} UNumberFormatRoundingMode;

typedef enum UNumberFormatPadPosition {
    UNUM_PAD_BEFORE_PREFIX,
    UNUM_PAD_AFTER_PREFIX,
    UNUM_PAD_BEFORE_SUFFIX,
    UNUM_PAD_AFTER_SUFFIX
} UNumberFormatPadPosition;

U_STABLE UNumberFormat* U_EXPORT2 
unum_open(  UNumberFormatStyle    style,
            const    UChar*    pattern,
            int32_t            patternLength,
            const    char*     locale,
            UParseError*       parseErr,
            UErrorCode*        status);


U_STABLE void U_EXPORT2 
unum_close(UNumberFormat* fmt);

U_STABLE UNumberFormat* U_EXPORT2 
unum_clone(const UNumberFormat *fmt,
       UErrorCode *status);

U_STABLE int32_t U_EXPORT2 
unum_format(    const    UNumberFormat*    fmt,
        int32_t            number,
        UChar*            result,
        int32_t            resultLength,
        UFieldPosition    *pos,
        UErrorCode*        status);

U_STABLE int32_t U_EXPORT2 
unum_formatInt64(const UNumberFormat *fmt,
        int64_t         number,
        UChar*          result,
        int32_t         resultLength,
        UFieldPosition *pos,
        UErrorCode*     status);

U_STABLE int32_t U_EXPORT2 
unum_formatDouble(    const    UNumberFormat*  fmt,
            double          number,
            UChar*          result,
            int32_t         resultLength,
            UFieldPosition  *pos, /* 0 if ignore */
            UErrorCode*     status);

U_STABLE int32_t U_EXPORT2 
unum_formatDoubleCurrency(const UNumberFormat* fmt,
                          double number,
                          UChar* currency,
                          UChar* result,
                          int32_t resultLength,
                          UFieldPosition* pos, /* ignored if 0 */
                          UErrorCode* status);

U_STABLE int32_t U_EXPORT2 
unum_parse(    const   UNumberFormat*  fmt,
        const   UChar*          text,
        int32_t         textLength,
        int32_t         *parsePos /* 0 = start */,
        UErrorCode      *status);

U_STABLE int64_t U_EXPORT2 
unum_parseInt64(const UNumberFormat*  fmt,
        const UChar*  text,
        int32_t       textLength,
        int32_t       *parsePos /* 0 = start */,
        UErrorCode    *status);

U_STABLE double U_EXPORT2 
unum_parseDouble(    const   UNumberFormat*  fmt,
            const   UChar*          text,
            int32_t         textLength,
            int32_t         *parsePos /* 0 = start */,
            UErrorCode      *status);

U_STABLE double U_EXPORT2
unum_parseDoubleCurrency(const UNumberFormat* fmt,
                         const UChar* text,
                         int32_t textLength,
                         int32_t* parsePos, /* 0 = start */
                         UChar* currency,
                         UErrorCode* status);

U_STABLE void U_EXPORT2 
unum_applyPattern(          UNumberFormat  *format,
                            UBool          localized,
                    const   UChar          *pattern,
                            int32_t         patternLength,
                            UParseError    *parseError,
                            UErrorCode     *status
                                    );

U_STABLE const char* U_EXPORT2 
unum_getAvailable(int32_t localeIndex);

U_STABLE int32_t U_EXPORT2 
unum_countAvailable(void);

/** The possible UNumberFormat numeric attributes @stable ICU 2.0 */
typedef enum UNumberFormatAttribute {
  /** Parse integers only */
  UNUM_PARSE_INT_ONLY,
  /** Use grouping separator */
  UNUM_GROUPING_USED,
  /** Always show decimal point */
  UNUM_DECIMAL_ALWAYS_SHOWN,
  /** Maximum integer digits */
  UNUM_MAX_INTEGER_DIGITS,
  /** Minimum integer digits */
  UNUM_MIN_INTEGER_DIGITS,
  /** Integer digits */
  UNUM_INTEGER_DIGITS,
  /** Maximum fraction digits */
  UNUM_MAX_FRACTION_DIGITS,
  /** Minimum fraction digits */
  UNUM_MIN_FRACTION_DIGITS,
  /** Fraction digits */
  UNUM_FRACTION_DIGITS,
  /** Multiplier */
  UNUM_MULTIPLIER,
  /** Grouping size */
  UNUM_GROUPING_SIZE,
  /** Rounding Mode */
  UNUM_ROUNDING_MODE,
  /** Rounding increment */
  UNUM_ROUNDING_INCREMENT,
  /** The width to which the output of <code>format()</code> is padded. */
  UNUM_FORMAT_WIDTH,
  /** The position at which padding will take place. */
  UNUM_PADDING_POSITION,
  /** Secondary grouping size */
  UNUM_SECONDARY_GROUPING_SIZE,
  /** Use significant digits
   * @stable ICU 3.0 */
  UNUM_SIGNIFICANT_DIGITS_USED,
  /** Minimum significant digits
   * @stable ICU 3.0 */
  UNUM_MIN_SIGNIFICANT_DIGITS,
  /** Maximum significant digits
   * @stable ICU 3.0 */
  UNUM_MAX_SIGNIFICANT_DIGITS,
  /** Lenient parse mode used by rule-based formats.
   * @stable ICU 3.0
   */
  UNUM_LENIENT_PARSE
} UNumberFormatAttribute;

U_STABLE int32_t U_EXPORT2 
unum_getAttribute(const UNumberFormat*          fmt,
          UNumberFormatAttribute  attr);

U_STABLE void U_EXPORT2 
unum_setAttribute(    UNumberFormat*          fmt,
            UNumberFormatAttribute  attr,
            int32_t                 newValue);


U_STABLE double U_EXPORT2 
unum_getDoubleAttribute(const UNumberFormat*          fmt,
          UNumberFormatAttribute  attr);

U_STABLE void U_EXPORT2 
unum_setDoubleAttribute(    UNumberFormat*          fmt,
            UNumberFormatAttribute  attr,
            double                 newValue);

/** The possible UNumberFormat text attributes @stable ICU 2.0*/
typedef enum UNumberFormatTextAttribute {
  /** Positive prefix */
  UNUM_POSITIVE_PREFIX,
  /** Positive suffix */
  UNUM_POSITIVE_SUFFIX,
  /** Negative prefix */
  UNUM_NEGATIVE_PREFIX,
  /** Negative suffix */
  UNUM_NEGATIVE_SUFFIX,
  /** The character used to pad to the format width. */
  UNUM_PADDING_CHARACTER,
  /** The ISO currency code */
  UNUM_CURRENCY_CODE,
  /**
   * The default rule set.  This is only available with rule-based formatters.
   * @stable ICU 3.0
   */
  UNUM_DEFAULT_RULESET,
  /**
   * The public rule sets.  This is only available with rule-based formatters.
   * This is a read-only attribute.  The public rulesets are returned as a
   * single string, with each ruleset name delimited by ';' (semicolon).
   * @stable ICU 3.0
   */
  UNUM_PUBLIC_RULESETS
} UNumberFormatTextAttribute;

U_STABLE int32_t U_EXPORT2 
unum_getTextAttribute(    const    UNumberFormat*                    fmt,
            UNumberFormatTextAttribute      tag,
            UChar*                            result,
            int32_t                            resultLength,
            UErrorCode*                        status);

U_STABLE void U_EXPORT2 
unum_setTextAttribute(    UNumberFormat*                    fmt,
            UNumberFormatTextAttribute      tag,
            const    UChar*                            newValue,
            int32_t                            newValueLength,
            UErrorCode                        *status);

U_STABLE int32_t U_EXPORT2 
unum_toPattern(    const    UNumberFormat*          fmt,
        UBool                  isPatternLocalized,
        UChar*                  result,
        int32_t                 resultLength,
        UErrorCode*             status);


typedef enum UNumberFormatSymbol {
  /** The decimal separator */
  UNUM_DECIMAL_SEPARATOR_SYMBOL = 0,
  /** The grouping separator */
  UNUM_GROUPING_SEPARATOR_SYMBOL = 1,
  /** The pattern separator */
  UNUM_PATTERN_SEPARATOR_SYMBOL = 2,
  /** The percent sign */
  UNUM_PERCENT_SYMBOL = 3,
  /** Zero*/
  UNUM_ZERO_DIGIT_SYMBOL = 4,
  /** Character representing a digit in the pattern */
  UNUM_DIGIT_SYMBOL = 5,
  /** The minus sign */
  UNUM_MINUS_SIGN_SYMBOL = 6,
  /** The plus sign */
  UNUM_PLUS_SIGN_SYMBOL = 7,
  /** The currency symbol */
  UNUM_CURRENCY_SYMBOL = 8,
  /** The international currency symbol */
  UNUM_INTL_CURRENCY_SYMBOL = 9,
  /** The monetary separator */
  UNUM_MONETARY_SEPARATOR_SYMBOL = 10,
  /** The exponential symbol */
  UNUM_EXPONENTIAL_SYMBOL = 11,
  /** Per mill symbol */
  UNUM_PERMILL_SYMBOL = 12,
  /** Escape padding character */
  UNUM_PAD_ESCAPE_SYMBOL = 13,
  /** Infinity symbol */
  UNUM_INFINITY_SYMBOL = 14,
  /** Nan symbol */
  UNUM_NAN_SYMBOL = 15,
  /** Significant digit symbol
   * @stable ICU 3.0 */
  UNUM_SIGNIFICANT_DIGIT_SYMBOL = 16,
  /** The monetary grouping separator 
   * @stable ICU 3.6
   */
  UNUM_MONETARY_GROUPING_SEPARATOR_SYMBOL = 17,  
  /** count symbol constants */
  UNUM_FORMAT_SYMBOL_COUNT = 18
} UNumberFormatSymbol;

U_STABLE int32_t U_EXPORT2
unum_getSymbol(const UNumberFormat *fmt,
               UNumberFormatSymbol symbol,
               UChar *buffer,
               int32_t size,
               UErrorCode *status);

U_STABLE void U_EXPORT2
unum_setSymbol(UNumberFormat *fmt,
               UNumberFormatSymbol symbol,
               const UChar *value,
               int32_t length,
               UErrorCode *status);


U_STABLE const char* U_EXPORT2
unum_getLocaleByType(const UNumberFormat *fmt,
                     ULocDataLocaleType type,
                     UErrorCode* status); 

#endif /* #if !UCONFIG_NO_FORMATTING */

#endif
