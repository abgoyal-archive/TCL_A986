
#ifndef UDAT_H
#define UDAT_H

#include "unicode/utypes.h"

#if !UCONFIG_NO_FORMATTING

#include "unicode/localpointer.h"
#include "unicode/ucal.h"
#include "unicode/unum.h"

typedef void* UDateFormat;

typedef enum UDateFormatStyle {
    /** Full style */
    UDAT_FULL,
    /** Long style */
    UDAT_LONG,
    /** Medium style */
    UDAT_MEDIUM,
    /** Short style */
    UDAT_SHORT,
    /** Default style */
    UDAT_DEFAULT = UDAT_MEDIUM,

    /** Bitfield for relative date */
    UDAT_RELATIVE = (1 << 7),
    
    UDAT_FULL_RELATIVE = UDAT_FULL | UDAT_RELATIVE,
        
    UDAT_LONG_RELATIVE = UDAT_LONG | UDAT_RELATIVE,
    
    UDAT_MEDIUM_RELATIVE = UDAT_MEDIUM | UDAT_RELATIVE,
    
    UDAT_SHORT_RELATIVE = UDAT_SHORT | UDAT_RELATIVE,
    
    
    /** No style */
    UDAT_NONE = -1,
    /** for internal API use only */
    UDAT_IGNORE = -2

} UDateFormatStyle;



#define UDAT_MINUTE_SECOND              "ms"
#define UDAT_HOUR24_MINUTE              "Hm"
#define UDAT_HOUR24_MINUTE_SECOND       "Hms"      
#define UDAT_HOUR_MINUTE_SECOND         "hms"
#define UDAT_STANDALONE_MONTH           "LLLL"
#define UDAT_ABBR_STANDALONE_MONTH      "LLL"
#define UDAT_YEAR_QUARTER               "yQQQ"
#define UDAT_YEAR_ABBR_QUARTER          "yQ"

/** @} */

#define UDAT_HOUR_MINUTE                "hm"
#define UDAT_YEAR                       "y"
#define UDAT_DAY                        "d"
#define UDAT_NUM_MONTH_WEEKDAY_DAY      "MEd"
#define UDAT_YEAR_NUM_MONTH             "yM"              
#define UDAT_NUM_MONTH_DAY              "Md"
#define UDAT_YEAR_NUM_MONTH_WEEKDAY_DAY "yMEd"
#define UDAT_ABBR_MONTH_WEEKDAY_DAY     "MMMEd"
#define UDAT_YEAR_MONTH                 "yMMMM"
#define UDAT_YEAR_ABBR_MONTH            "yMMM"
#define UDAT_MONTH_DAY                  "MMMMd"
#define UDAT_ABBR_MONTH_DAY             "MMMd" 
#define UDAT_MONTH_WEEKDAY_DAY          "MMMMEEEEd"
#define UDAT_YEAR_ABBR_MONTH_WEEKDAY_DAY "yMMMEd" 
#define UDAT_YEAR_MONTH_WEEKDAY_DAY     "yMMMMEEEEd"
#define UDAT_YEAR_MONTH_DAY             "yMMMMd"
#define UDAT_YEAR_ABBR_MONTH_DAY        "yMMMd"
#define UDAT_YEAR_NUM_MONTH_DAY         "yMd"
#define UDAT_NUM_MONTH                  "M"
#define UDAT_ABBR_MONTH                 "MMM"
#define UDAT_MONTH                      "MMMM"
#define UDAT_HOUR_MINUTE_GENERIC_TZ     "hmv"
#define UDAT_HOUR_MINUTE_TZ             "hmz"
#define UDAT_HOUR                       "h"
#define UDAT_HOUR_GENERIC_TZ            "hv"
#define UDAT_HOUR_TZ                    "hz"

/** @} */


typedef enum UDateFormatField {
    /**
     * FieldPosition and UFieldPosition selector for 'G' field alignment,
     * corresponding to the UCAL_ERA field.
     * @stable ICU 3.0
     */
    UDAT_ERA_FIELD = 0,

    /**
     * FieldPosition and UFieldPosition selector for 'y' field alignment,
     * corresponding to the UCAL_YEAR field.
     * @stable ICU 3.0
     */
    UDAT_YEAR_FIELD = 1,

    /**
     * FieldPosition and UFieldPosition selector for 'M' field alignment,
     * corresponding to the UCAL_MONTH field.
     * @stable ICU 3.0
     */
    UDAT_MONTH_FIELD = 2,

    /**
     * FieldPosition and UFieldPosition selector for 'd' field alignment,
     * corresponding to the UCAL_DATE field.
     * @stable ICU 3.0
     */
    UDAT_DATE_FIELD = 3,

    /**
     * FieldPosition and UFieldPosition selector for 'k' field alignment,
     * corresponding to the UCAL_HOUR_OF_DAY field.
     * UDAT_HOUR_OF_DAY1_FIELD is used for the one-based 24-hour clock.
     * For example, 23:59 + 01:00 results in 24:59.
     * @stable ICU 3.0
     */
    UDAT_HOUR_OF_DAY1_FIELD = 4,

    /**
     * FieldPosition and UFieldPosition selector for 'H' field alignment,
     * corresponding to the UCAL_HOUR_OF_DAY field.
     * UDAT_HOUR_OF_DAY0_FIELD is used for the zero-based 24-hour clock.
     * For example, 23:59 + 01:00 results in 00:59.
     * @stable ICU 3.0
     */
    UDAT_HOUR_OF_DAY0_FIELD = 5,

    /**
     * FieldPosition and UFieldPosition selector for 'm' field alignment,
     * corresponding to the UCAL_MINUTE field.
     * @stable ICU 3.0
     */
    UDAT_MINUTE_FIELD = 6,

    /**
     * FieldPosition and UFieldPosition selector for 's' field alignment,
     * corresponding to the UCAL_SECOND field.
     * @stable ICU 3.0
     */
    UDAT_SECOND_FIELD = 7,

    /**
     * FieldPosition and UFieldPosition selector for 'S' field alignment,
     * corresponding to the UCAL_MILLISECOND field.
     * @stable ICU 3.0
     */
    UDAT_FRACTIONAL_SECOND_FIELD = 8,

    /**
     * FieldPosition and UFieldPosition selector for 'E' field alignment,
     * corresponding to the UCAL_DAY_OF_WEEK field.
     * @stable ICU 3.0
     */
    UDAT_DAY_OF_WEEK_FIELD = 9,

    /**
     * FieldPosition and UFieldPosition selector for 'D' field alignment,
     * corresponding to the UCAL_DAY_OF_YEAR field.
     * @stable ICU 3.0
     */
    UDAT_DAY_OF_YEAR_FIELD = 10,

    /**
     * FieldPosition and UFieldPosition selector for 'F' field alignment,
     * corresponding to the UCAL_DAY_OF_WEEK_IN_MONTH field.
     * @stable ICU 3.0
     */
    UDAT_DAY_OF_WEEK_IN_MONTH_FIELD = 11,

    /**
     * FieldPosition and UFieldPosition selector for 'w' field alignment,
     * corresponding to the UCAL_WEEK_OF_YEAR field.
     * @stable ICU 3.0
     */
    UDAT_WEEK_OF_YEAR_FIELD = 12,

    /**
     * FieldPosition and UFieldPosition selector for 'W' field alignment,
     * corresponding to the UCAL_WEEK_OF_MONTH field.
     * @stable ICU 3.0
     */
    UDAT_WEEK_OF_MONTH_FIELD = 13,

    /**
     * FieldPosition and UFieldPosition selector for 'a' field alignment,
     * corresponding to the UCAL_AM_PM field.
     * @stable ICU 3.0
     */
    UDAT_AM_PM_FIELD = 14,

    /**
     * FieldPosition and UFieldPosition selector for 'h' field alignment,
     * corresponding to the UCAL_HOUR field.
     * UDAT_HOUR1_FIELD is used for the one-based 12-hour clock.
     * For example, 11:30 PM + 1 hour results in 12:30 AM.
     * @stable ICU 3.0
     */
    UDAT_HOUR1_FIELD = 15,

    /**
     * FieldPosition and UFieldPosition selector for 'K' field alignment,
     * corresponding to the UCAL_HOUR field.
     * UDAT_HOUR0_FIELD is used for the zero-based 12-hour clock.
     * For example, 11:30 PM + 1 hour results in 00:30 AM.
     * @stable ICU 3.0
     */
    UDAT_HOUR0_FIELD = 16,

    /**
     * FieldPosition and UFieldPosition selector for 'z' field alignment,
     * corresponding to the UCAL_ZONE_OFFSET and
     * UCAL_DST_OFFSET fields.
     * @stable ICU 3.0
     */
    UDAT_TIMEZONE_FIELD = 17,

    /**
     * FieldPosition and UFieldPosition selector for 'Y' field alignment,
     * corresponding to the UCAL_YEAR_WOY field.
     * @stable ICU 3.0
     */
    UDAT_YEAR_WOY_FIELD = 18,

    /**
     * FieldPosition and UFieldPosition selector for 'e' field alignment,
     * corresponding to the UCAL_DOW_LOCAL field.
     * @stable ICU 3.0
     */
    UDAT_DOW_LOCAL_FIELD = 19,

    /**
     * FieldPosition and UFieldPosition selector for 'u' field alignment,
     * corresponding to the UCAL_EXTENDED_YEAR field.
     * @stable ICU 3.0
     */
    UDAT_EXTENDED_YEAR_FIELD = 20,

    /**
     * FieldPosition and UFieldPosition selector for 'g' field alignment,
     * corresponding to the UCAL_JULIAN_DAY field.
     * @stable ICU 3.0
     */
    UDAT_JULIAN_DAY_FIELD = 21,

    /**
     * FieldPosition and UFieldPosition selector for 'A' field alignment,
     * corresponding to the UCAL_MILLISECONDS_IN_DAY field.
     * @stable ICU 3.0
     */
    UDAT_MILLISECONDS_IN_DAY_FIELD = 22,

    /**
     * FieldPosition and UFieldPosition selector for 'Z' field alignment,
     * corresponding to the UCAL_ZONE_OFFSET and
     * UCAL_DST_OFFSET fields.
     * @stable ICU 3.0
     */
    UDAT_TIMEZONE_RFC_FIELD = 23,

    /**
     * FieldPosition and UFieldPosition selector for 'v' field alignment,
     * corresponding to the UCAL_ZONE_OFFSET field.
     * @stable ICU 3.4
     */
    UDAT_TIMEZONE_GENERIC_FIELD = 24,
    /**
     * FieldPosition selector for 'c' field alignment,
     * corresponding to the {@link #UCAL_DOW_LOCAL} field.
     * This displays the stand alone day name, if available.
     * @stable ICU 3.4
     */
    UDAT_STANDALONE_DAY_FIELD = 25,

    /**
     * FieldPosition selector for 'L' field alignment,
     * corresponding to the {@link #UCAL_MONTH} field.
     * This displays the stand alone month name, if available.
     * @stable ICU 3.4
     */
    UDAT_STANDALONE_MONTH_FIELD = 26,

    /**
     * FieldPosition selector for "Q" field alignment,
     * corresponding to quarters. This is implemented
     * using the {@link #UCAL_MONTH} field. This
     * displays the quarter.
     * @stable ICU 3.6
     */
    UDAT_QUARTER_FIELD = 27,

    /**
     * FieldPosition selector for the "q" field alignment,
     * corresponding to stand-alone quarters. This is
     * implemented using the {@link #UCAL_MONTH} field.
     * This displays the stand-alone quarter.
     * @stable ICU 3.6
     */
    UDAT_STANDALONE_QUARTER_FIELD = 28,

    /**
     * FieldPosition and UFieldPosition selector for 'V' field alignment,
     * corresponding to the UCAL_ZONE_OFFSET field.
     * @stable ICU 3.8
     */
    UDAT_TIMEZONE_SPECIAL_FIELD = 29,

   /**
     * Number of FieldPosition and UFieldPosition selectors for
     * DateFormat and UDateFormat.
     * Valid selectors range from 0 to UDAT_FIELD_COUNT-1.
     * This value is subject to change if new fields are defined
     * in the future.
     * @stable ICU 3.0
     */
    UDAT_FIELD_COUNT = 30

} UDateFormatField;


U_DRAFT UCalendarDateFields U_EXPORT2
udat_toCalendarDateField(UDateFormatField field);


U_STABLE UDateFormat* U_EXPORT2 
udat_open(UDateFormatStyle  timeStyle,
          UDateFormatStyle  dateStyle,
          const char        *locale,
          const UChar       *tzID,
          int32_t           tzIDLength,
          const UChar       *pattern,
          int32_t           patternLength,
          UErrorCode        *status);


U_STABLE void U_EXPORT2 
udat_close(UDateFormat* format);

#if U_SHOW_CPLUSPLUS_API

U_NAMESPACE_BEGIN

U_DEFINE_LOCAL_OPEN_POINTER(LocalUDateFormatPointer, UDateFormat, udat_close);

U_NAMESPACE_END

#endif

U_STABLE UDateFormat* U_EXPORT2 
udat_clone(const UDateFormat *fmt,
       UErrorCode *status);

U_STABLE int32_t U_EXPORT2 
udat_format(    const    UDateFormat*    format,
                        UDate           dateToFormat,
                        UChar*          result,
                        int32_t         resultLength,
                        UFieldPosition* position,
                        UErrorCode*     status);

U_STABLE UDate U_EXPORT2 
udat_parse(    const    UDateFormat*    format,
            const    UChar*          text,
                    int32_t         textLength,
                    int32_t         *parsePos,
                    UErrorCode      *status);

U_STABLE void U_EXPORT2 
udat_parseCalendar(const    UDateFormat*    format,
                            UCalendar*      calendar,
                   const    UChar*          text,
                            int32_t         textLength,
                            int32_t         *parsePos,
                            UErrorCode      *status);

U_STABLE UBool U_EXPORT2 
udat_isLenient(const UDateFormat* fmt);

U_STABLE void U_EXPORT2 
udat_setLenient(    UDateFormat*    fmt,
                    UBool          isLenient);

U_STABLE const UCalendar* U_EXPORT2 
udat_getCalendar(const UDateFormat* fmt);

U_STABLE void U_EXPORT2 
udat_setCalendar(            UDateFormat*    fmt,
                    const   UCalendar*      calendarToSet);

U_STABLE const UNumberFormat* U_EXPORT2 
udat_getNumberFormat(const UDateFormat* fmt);

U_STABLE void U_EXPORT2 
udat_setNumberFormat(            UDateFormat*    fmt,
                        const   UNumberFormat*  numberFormatToSet);

U_STABLE const char* U_EXPORT2 
udat_getAvailable(int32_t localeIndex);

U_STABLE int32_t U_EXPORT2 
udat_countAvailable(void);

U_STABLE UDate U_EXPORT2 
udat_get2DigitYearStart(    const   UDateFormat     *fmt,
                                    UErrorCode      *status);

U_STABLE void U_EXPORT2 
udat_set2DigitYearStart(    UDateFormat     *fmt,
                            UDate           d,
                            UErrorCode      *status);

U_STABLE int32_t U_EXPORT2 
udat_toPattern(    const   UDateFormat     *fmt,
                        UBool          localized,
                        UChar           *result,
                        int32_t         resultLength,
                        UErrorCode      *status);

U_STABLE void U_EXPORT2 
udat_applyPattern(            UDateFormat     *format,
                            UBool          localized,
                    const   UChar           *pattern,
                            int32_t         patternLength);

typedef enum UDateFormatSymbolType {
    /** The era names, for example AD */
    UDAT_ERAS,
    /** The month names, for example February */
    UDAT_MONTHS,
    /** The short month names, for example Feb. */
    UDAT_SHORT_MONTHS,
    /** The weekday names, for example Monday */
    UDAT_WEEKDAYS,
    /** The short weekday names, for example Mon. */
    UDAT_SHORT_WEEKDAYS,
    /** The AM/PM names, for example AM */
    UDAT_AM_PMS,
    /** The localized characters */
    UDAT_LOCALIZED_CHARS,
    /** The long era names, for example Anno Domini */
    UDAT_ERA_NAMES,
    /** The narrow month names, for example F */
    UDAT_NARROW_MONTHS,
    /** The narrow weekday names, for example N */
    UDAT_NARROW_WEEKDAYS,
    /** Standalone context versions of months */
    UDAT_STANDALONE_MONTHS,
    UDAT_STANDALONE_SHORT_MONTHS,
    UDAT_STANDALONE_NARROW_MONTHS,
    /** Standalone context versions of weekdays */
    UDAT_STANDALONE_WEEKDAYS,
    UDAT_STANDALONE_SHORT_WEEKDAYS,
    UDAT_STANDALONE_NARROW_WEEKDAYS,
    /** The quarters, for example 1st Quarter */
    UDAT_QUARTERS,
    /** The short quarter names, for example Q1 */
    UDAT_SHORT_QUARTERS,
    /** Standalone context versions of quarters */
    UDAT_STANDALONE_QUARTERS,
    UDAT_STANDALONE_SHORT_QUARTERS

} UDateFormatSymbolType;

struct UDateFormatSymbols;
typedef struct UDateFormatSymbols UDateFormatSymbols;

U_STABLE int32_t U_EXPORT2 
udat_getSymbols(const   UDateFormat             *fmt,
                        UDateFormatSymbolType   type,
                        int32_t                 symbolIndex,
                        UChar                   *result,
                        int32_t                 resultLength,
                        UErrorCode              *status);

U_STABLE int32_t U_EXPORT2 
udat_countSymbols(    const    UDateFormat                *fmt,
                            UDateFormatSymbolType    type);

U_STABLE void U_EXPORT2 
udat_setSymbols(    UDateFormat             *format,
                    UDateFormatSymbolType   type,
                    int32_t                 symbolIndex,
                    UChar                   *value,
                    int32_t                 valueLength,
                    UErrorCode              *status);

U_STABLE const char* U_EXPORT2
udat_getLocaleByType(const UDateFormat *fmt,
                     ULocDataLocaleType type,
                     UErrorCode* status); 

U_INTERNAL int32_t U_EXPORT2 
udat_toPatternRelativeDate(const UDateFormat *fmt,
                           UChar             *result,
                           int32_t           resultLength,
                           UErrorCode        *status);

U_INTERNAL int32_t U_EXPORT2 
udat_toPatternRelativeTime(const UDateFormat *fmt,
                           UChar             *result,
                           int32_t           resultLength,
                           UErrorCode        *status);

U_INTERNAL void U_EXPORT2 
udat_applyPatternRelative(UDateFormat *format,
                          const UChar *datePattern,
                          int32_t     datePatternLength,
                          const UChar *timePattern,
                          int32_t     timePatternLength,
                          UErrorCode  *status);


#endif /* #if !UCONFIG_NO_FORMATTING */

#endif
