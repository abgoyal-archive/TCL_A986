
#ifndef UCAL_H
#define UCAL_H

#include "unicode/utypes.h"
#include "unicode/uenum.h"
#include "unicode/uloc.h"

#if !UCONFIG_NO_FORMATTING


typedef void* UCalendar;

enum UCalendarType {
  /**
   * Despite the name, UCAL_TRADITIONAL designates the locale's default calendar,
   * which may be the Gregorian calendar or some other calendar.
   * @stable ICU 2.0
   */
  UCAL_TRADITIONAL,
  /**
   * Unambiguously designates the Gregorian calendar for the locale.
   * @stable ICU 2.0
   */
  UCAL_GREGORIAN,
  /**
   * A better name for UCAL_TRADITIONAL.
   * @draft ICU 4.2
   */
  UCAL_DEFAULT = UCAL_TRADITIONAL
};

/** @stable ICU 2.0 */
typedef enum UCalendarType UCalendarType;

enum UCalendarDateFields {
  /** 
   * Field number indicating the era, e.g., AD or BC in the Gregorian (Julian) calendar. 
   * This is a calendar-specific value.
   * @stable ICU 2.6 
   */
  UCAL_ERA,

  /**
   * Field number indicating the year. This is a calendar-specific value.
   * @stable ICU 2.6 
   */
  UCAL_YEAR,

  /**
   * Field number indicating the month. This is a calendar-specific value. 
   * The first month of the year is
   * <code>JANUARY</code>; the last depends on the number of months in a year.
   * @see #UCAL_JANUARY
   * @see #UCAL_FEBRUARY
   * @see #UCAL_MARCH
   * @see #UCAL_APRIL
   * @see #UCAL_MAY
   * @see #UCAL_JUNE
   * @see #UCAL_JULY
   * @see #UCAL_AUGUST
   * @see #UCAL_SEPTEMBER
   * @see #UCAL_OCTOBER
   * @see #UCAL_NOVEMBER
   * @see #UCAL_DECEMBER
   * @see #UCAL_UNDECIMBER
   * @stable ICU 2.6 
   */
  UCAL_MONTH,

  /**
   * Field number indicating the
   * week number within the current year.  The first week of the year, as
   * defined by <code>UCAL_FIRST_DAY_OF_WEEK</code> and <code>UCAL_MINIMAL_DAYS_IN_FIRST_WEEK</code>
   * attributes, has value 1.  Subclasses define
   * the value of <code>UCAL_WEEK_OF_YEAR</code> for days before the first week of
   * the year.
   * @see ucal_getAttribute
   * @see ucal_setAttribute
   * @stable ICU 2.6 
   */
  UCAL_WEEK_OF_YEAR,

 /**
   * Field number indicating the
   * week number within the current month.  The first week of the month, as
   * defined by <code>UCAL_FIRST_DAY_OF_WEEK</code> and <code>UCAL_MINIMAL_DAYS_IN_FIRST_WEEK</code>
   * attributes, has value 1.  Subclasses define
   * the value of <code>WEEK_OF_MONTH</code> for days before the first week of
   * the month.
   * @see ucal_getAttribute
   * @see ucal_setAttribute
   * @see #UCAL_FIRST_DAY_OF_WEEK
   * @see #UCAL_MINIMAL_DAYS_IN_FIRST_WEEK
   * @stable ICU 2.6 
   */
  UCAL_WEEK_OF_MONTH,

 /**
   * Field number indicating the
   * day of the month. This is a synonym for <code>DAY_OF_MONTH</code>.
   * The first day of the month has value 1.
   * @see #UCAL_DAY_OF_MONTH
   * @stable ICU 2.6 
   */
  UCAL_DATE,

 /**
   * Field number indicating the day
   * number within the current year.  The first day of the year has value 1.
   * @stable ICU 2.6 
   */
  UCAL_DAY_OF_YEAR,

 /**
   * Field number indicating the day
   * of the week.  This field takes values <code>SUNDAY</code>,
   * <code>MONDAY</code>, <code>TUESDAY</code>, <code>WEDNESDAY</code>,
   * <code>THURSDAY</code>, <code>FRIDAY</code>, and <code>SATURDAY</code>.
   * @see #UCAL_SUNDAY
   * @see #UCAL_MONDAY
   * @see #UCAL_TUESDAY
   * @see #UCAL_WEDNESDAY
   * @see #UCAL_THURSDAY
   * @see #UCAL_FRIDAY
   * @see #UCAL_SATURDAY
   * @stable ICU 2.6 
   */
  UCAL_DAY_OF_WEEK,

 /**
   * Field number indicating the
   * ordinal number of the day of the week within the current month. Together
   * with the <code>DAY_OF_WEEK</code> field, this uniquely specifies a day
   * within a month.  Unlike <code>WEEK_OF_MONTH</code> and
   * <code>WEEK_OF_YEAR</code>, this field's value does <em>not</em> depend on
   * <code>getFirstDayOfWeek()</code> or
   * <code>getMinimalDaysInFirstWeek()</code>.  <code>DAY_OF_MONTH 1</code>
   * through <code>7</code> always correspond to <code>DAY_OF_WEEK_IN_MONTH
   * 1</code>; <code>8</code> through <code>15</code> correspond to
   * <code>DAY_OF_WEEK_IN_MONTH 2</code>, and so on.
   * <code>DAY_OF_WEEK_IN_MONTH 0</code> indicates the week before
   * <code>DAY_OF_WEEK_IN_MONTH 1</code>.  Negative values count back from the
   * end of the month, so the last Sunday of a month is specified as
   * <code>DAY_OF_WEEK = SUNDAY, DAY_OF_WEEK_IN_MONTH = -1</code>.  Because
   * negative values count backward they will usually be aligned differently
   * within the month than positive values.  For example, if a month has 31
   * days, <code>DAY_OF_WEEK_IN_MONTH -1</code> will overlap
   * <code>DAY_OF_WEEK_IN_MONTH 5</code> and the end of <code>4</code>.
   * @see #UCAL_DAY_OF_WEEK
   * @see #UCAL_WEEK_OF_MONTH
   * @stable ICU 2.6 
   */
  UCAL_DAY_OF_WEEK_IN_MONTH,

 /**
   * Field number indicating
   * whether the <code>HOUR</code> is before or after noon.
   * E.g., at 10:04:15.250 PM the <code>AM_PM</code> is <code>PM</code>.
   * @see #UCAL_AM
   * @see #UCAL_PM
   * @see #UCAL_HOUR
   * @stable ICU 2.6 
   */
  UCAL_AM_PM,

 /**
   * Field number indicating the
   * hour of the morning or afternoon. <code>HOUR</code> is used for the 12-hour
   * clock.
   * E.g., at 10:04:15.250 PM the <code>HOUR</code> is 10.
   * @see #UCAL_AM_PM
   * @see #UCAL_HOUR_OF_DAY
   * @stable ICU 2.6 
   */
  UCAL_HOUR,

 /**
   * Field number indicating the
   * hour of the day. <code>HOUR_OF_DAY</code> is used for the 24-hour clock.
   * E.g., at 10:04:15.250 PM the <code>HOUR_OF_DAY</code> is 22.
   * @see #UCAL_HOUR
   * @stable ICU 2.6 
   */
  UCAL_HOUR_OF_DAY,

 /**
   * Field number indicating the
   * minute within the hour.
   * E.g., at 10:04:15.250 PM the <code>UCAL_MINUTE</code> is 4.
   * @stable ICU 2.6 
   */
  UCAL_MINUTE,

 /**
   * Field number indicating the
   * second within the minute.
   * E.g., at 10:04:15.250 PM the <code>UCAL_SECOND</code> is 15.
   * @stable ICU 2.6 
   */
  UCAL_SECOND,

 /**
   * Field number indicating the
   * millisecond within the second.
   * E.g., at 10:04:15.250 PM the <code>UCAL_MILLISECOND</code> is 250.
   * @stable ICU 2.6 
   */
  UCAL_MILLISECOND,

 /**
   * Field number indicating the
   * raw offset from GMT in milliseconds.
   * @stable ICU 2.6 
   */
  UCAL_ZONE_OFFSET,

 /**
   * Field number indicating the
   * daylight savings offset in milliseconds.
   * @stable ICU 2.6 
   */
  UCAL_DST_OFFSET,
  
 /**
   * Field number 
   * indicating the extended year corresponding to the
   * <code>UCAL_WEEK_OF_YEAR</code> field.  This may be one greater or less
   * than the value of <code>UCAL_EXTENDED_YEAR</code>.
   * @stable ICU 2.6
   */
  UCAL_YEAR_WOY,

 /**
   * Field number 
   * indicating the localized day of week.  This will be a value from 1
   * to 7 inclusive, with 1 being the localized first day of the week.
   * @stable ICU 2.6
   */
  UCAL_DOW_LOCAL,

  /**
   * Year of this calendar system, encompassing all supra-year fields. For example, 
   * in Gregorian/Julian calendars, positive Extended Year values indicate years AD,
   *  1 BC = 0 extended, 2 BC = -1 extended, and so on. 
   * @stable ICU 2.8 
   */
  UCAL_EXTENDED_YEAR,

 /**
   * Field number 
   * indicating the modified Julian day number.  This is different from
   * the conventional Julian day number in two regards.  First, it
   * demarcates days at local zone midnight, rather than noon GMT.
   * Second, it is a local number; that is, it depends on the local time
   * zone.  It can be thought of as a single number that encompasses all
   * the date-related fields.
   * @stable ICU 2.8
   */
  UCAL_JULIAN_DAY, 

  /**
   * Ranges from 0 to 23:59:59.999 (regardless of DST).  This field behaves <em>exactly</em> 
   * like a composite of all time-related fields, not including the zone fields.  As such, 
   * it also reflects discontinuities of those fields on DST transition days.  On a day
   * of DST onset, it will jump forward.  On a day of DST cessation, it will jump 
   * backward.  This reflects the fact that it must be combined with the DST_OFFSET field
   * to obtain a unique local time value.
   * @stable ICU 2.8
   */
  UCAL_MILLISECONDS_IN_DAY,

  /**
   * Whether or not the current month is a leap month (0 or 1). See the Chinese calendar for
   * an example of this.
   */
  UCAL_IS_LEAP_MONTH,
  
  /**
   * Field count
   * @stable ICU 2.6
   */
  UCAL_FIELD_COUNT,

 /**
   * Field number indicating the
   * day of the month. This is a synonym for <code>UCAL_DATE</code>.
   * The first day of the month has value 1.
   * @see #UCAL_DATE
   * Synonym for UCAL_DATE
   * @stable ICU 2.8
   **/
  UCAL_DAY_OF_MONTH=UCAL_DATE
};

/** @stable ICU 2.0 */
typedef enum UCalendarDateFields UCalendarDateFields;
    /**
     * Useful constant for days of week. Note: Calendar day-of-week is 1-based. Clients
     * who create locale resources for the field of first-day-of-week should be aware of
     * this. For instance, in US locale, first-day-of-week is set to 1, i.e., UCAL_SUNDAY.
     */
enum UCalendarDaysOfWeek {
  /** Sunday */
  UCAL_SUNDAY = 1,
  /** Monday */
  UCAL_MONDAY,
  /** Tuesday */
  UCAL_TUESDAY,
  /** Wednesday */
  UCAL_WEDNESDAY,
  /** Thursday */
  UCAL_THURSDAY,
  /** Friday */
  UCAL_FRIDAY,
  /** Saturday */
  UCAL_SATURDAY
};

/** @stable ICU 2.0 */
typedef enum UCalendarDaysOfWeek UCalendarDaysOfWeek;

enum UCalendarMonths {
  /** January */
  UCAL_JANUARY,
  /** February */
  UCAL_FEBRUARY,
  /** March */
  UCAL_MARCH,
  /** April */
  UCAL_APRIL,
  /** May */
  UCAL_MAY,
  /** June */
  UCAL_JUNE,
  /** July */
  UCAL_JULY,
  /** August */
  UCAL_AUGUST,
  /** September */
  UCAL_SEPTEMBER,
  /** October */
  UCAL_OCTOBER,
  /** November */
  UCAL_NOVEMBER,
  /** December */
  UCAL_DECEMBER,
  /** Value of the <code>UCAL_MONTH</code> field indicating the
    * thirteenth month of the year. Although the Gregorian calendar
    * does not use this value, lunar calendars do.
    */
  UCAL_UNDECIMBER
};

/** @stable ICU 2.0 */
typedef enum UCalendarMonths UCalendarMonths;

enum UCalendarAMPMs {
    /** AM */
  UCAL_AM,
  /** PM */
  UCAL_PM
};

/** @stable ICU 2.0 */
typedef enum UCalendarAMPMs UCalendarAMPMs;

U_STABLE UEnumeration* U_EXPORT2
ucal_openTimeZones(UErrorCode* ec);

U_STABLE UEnumeration* U_EXPORT2
ucal_openCountryTimeZones(const char* country, UErrorCode* ec);

U_STABLE int32_t U_EXPORT2
ucal_getDefaultTimeZone(UChar* result, int32_t resultCapacity, UErrorCode* ec);

U_STABLE void U_EXPORT2
ucal_setDefaultTimeZone(const UChar* zoneID, UErrorCode* ec);

U_STABLE int32_t U_EXPORT2
ucal_getDSTSavings(const UChar* zoneID, UErrorCode* ec);

U_STABLE UDate U_EXPORT2 
ucal_getNow(void);

U_STABLE UCalendar* U_EXPORT2 
ucal_open(const UChar*   zoneID,
          int32_t        len,
          const char*    locale,
          UCalendarType  type,
          UErrorCode*    status);

U_STABLE void U_EXPORT2 
ucal_close(UCalendar *cal);

U_DRAFT UCalendar* U_EXPORT2 
ucal_clone(const UCalendar* cal,
           UErrorCode*      status);

U_STABLE void U_EXPORT2 
ucal_setTimeZone(UCalendar*    cal,
                 const UChar*  zoneID,
                 int32_t       len,
                 UErrorCode*   status);

enum UCalendarDisplayNameType {
  /** Standard display name */
  UCAL_STANDARD,
  /** Short standard display name */
  UCAL_SHORT_STANDARD,
  /** Daylight savings display name */
  UCAL_DST,
  /** Short daylight savings display name */
  UCAL_SHORT_DST
};

/** @stable ICU 2.0 */
typedef enum UCalendarDisplayNameType UCalendarDisplayNameType;

U_STABLE int32_t U_EXPORT2 
ucal_getTimeZoneDisplayName(const UCalendar*          cal,
                            UCalendarDisplayNameType  type,
                            const char*               locale,
                            UChar*                    result,
                            int32_t                   resultLength,
                            UErrorCode*               status);

U_STABLE UBool U_EXPORT2 
ucal_inDaylightTime(const UCalendar*  cal,
                    UErrorCode*       status );

U_STABLE void U_EXPORT2
ucal_setGregorianChange(UCalendar *cal, UDate date, UErrorCode *pErrorCode);

U_STABLE UDate U_EXPORT2
ucal_getGregorianChange(const UCalendar *cal, UErrorCode *pErrorCode);

enum UCalendarAttribute {
    /** Lenient parsing */
  UCAL_LENIENT,
  /** First day of week */
  UCAL_FIRST_DAY_OF_WEEK,
  /** Minimum number of days in first week */
  UCAL_MINIMAL_DAYS_IN_FIRST_WEEK
};

/** @stable ICU 2.0 */
typedef enum UCalendarAttribute UCalendarAttribute;

U_STABLE int32_t U_EXPORT2 
ucal_getAttribute(const UCalendar*    cal,
                  UCalendarAttribute  attr);

U_STABLE void U_EXPORT2 
ucal_setAttribute(UCalendar*          cal,
                  UCalendarAttribute  attr,
                  int32_t             newValue);

U_STABLE const char* U_EXPORT2 
ucal_getAvailable(int32_t localeIndex);

U_STABLE int32_t U_EXPORT2 
ucal_countAvailable(void);

U_STABLE UDate U_EXPORT2 
ucal_getMillis(const UCalendar*  cal,
               UErrorCode*       status);

U_STABLE void U_EXPORT2 
ucal_setMillis(UCalendar*   cal,
               UDate        dateTime,
               UErrorCode*  status );

U_STABLE void U_EXPORT2 
ucal_setDate(UCalendar*   cal,
             int32_t      year,
             int32_t      month,
             int32_t      date,
             UErrorCode*  status);

U_STABLE void U_EXPORT2 
ucal_setDateTime(UCalendar*   cal,
                 int32_t      year,
                 int32_t      month,
                 int32_t      date,
                 int32_t      hour,
                 int32_t      minute,
                 int32_t      second,
                 UErrorCode*  status);

U_STABLE UBool U_EXPORT2 
ucal_equivalentTo(const UCalendar*  cal1,
                  const UCalendar*  cal2);

U_STABLE void U_EXPORT2 
ucal_add(UCalendar*           cal,
         UCalendarDateFields  field,
         int32_t              amount,
         UErrorCode*          status);

U_STABLE void U_EXPORT2 
ucal_roll(UCalendar*           cal,
          UCalendarDateFields  field,
          int32_t              amount,
          UErrorCode*          status);

U_STABLE int32_t U_EXPORT2 
ucal_get(const UCalendar*     cal,
         UCalendarDateFields  field,
         UErrorCode*          status );

U_STABLE void U_EXPORT2 
ucal_set(UCalendar*           cal,
         UCalendarDateFields  field,
         int32_t              value);

U_STABLE UBool U_EXPORT2 
ucal_isSet(const UCalendar*     cal,
           UCalendarDateFields  field);

U_STABLE void U_EXPORT2 
ucal_clearField(UCalendar*           cal,
                UCalendarDateFields  field);

U_STABLE void U_EXPORT2 
ucal_clear(UCalendar* calendar);

enum UCalendarLimitType {
  /** Minimum value */
  UCAL_MINIMUM,
  /** Maximum value */
  UCAL_MAXIMUM,
  /** Greatest minimum value */
  UCAL_GREATEST_MINIMUM,
  /** Leaest maximum value */
  UCAL_LEAST_MAXIMUM,
  /** Actual minimum value */
  UCAL_ACTUAL_MINIMUM,
  /** Actual maximum value */
  UCAL_ACTUAL_MAXIMUM
};

/** @stable ICU 2.0 */
typedef enum UCalendarLimitType UCalendarLimitType;

U_STABLE int32_t U_EXPORT2 
ucal_getLimit(const UCalendar*     cal,
              UCalendarDateFields  field,
              UCalendarLimitType   type,
              UErrorCode*          status);

U_STABLE const char * U_EXPORT2
ucal_getLocaleByType(const UCalendar *cal, ULocDataLocaleType type, UErrorCode* status);

U_DRAFT const char * U_EXPORT2
ucal_getTZDataVersion(UErrorCode* status);

U_DRAFT int32_t U_EXPORT2
ucal_getCanonicalTimeZoneID(const UChar* id, int32_t len,
                            UChar* result, int32_t resultCapacity, UBool *isSystemID, UErrorCode* status);
U_DRAFT const char * U_EXPORT2
ucal_getType(const UCalendar *cal, UErrorCode* status);

U_DRAFT UEnumeration* U_EXPORT2
ucal_getKeywordValuesForLocale(const char* key,
                               const char* locale,
                               UBool commonlyUsed,
                               UErrorCode* status);


#endif /* #if !UCONFIG_NO_FORMATTING */

#endif
