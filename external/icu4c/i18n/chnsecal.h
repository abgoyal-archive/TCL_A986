
#ifndef CHNSECAL_H
#define CHNSECAL_H

#include "unicode/utypes.h"

#if !UCONFIG_NO_FORMATTING

#include "unicode/calendar.h"

U_NAMESPACE_BEGIN

class ChineseCalendar : public Calendar {
 public:
  //-------------------------------------------------------------------------
  // Constructors...
  //-------------------------------------------------------------------------

  /**
   * Constructs an ChineseCalendar based on the current time in the default time zone
   * with the given locale.
   *
   * @param aLocale  The given locale.
   * @param success  Indicates the status of ChineseCalendar object construction.
   *                 Returns U_ZERO_ERROR if constructed successfully.
   * @internal
   */
  ChineseCalendar(const Locale& aLocale, UErrorCode &success);

  /**
   * Copy Constructor
   * @internal
   */
  ChineseCalendar(const ChineseCalendar& other);

  /**
   * Destructor.
   * @internal
   */
  virtual ~ChineseCalendar();

  // clone
  virtual Calendar* clone() const;

 private:

  //-------------------------------------------------------------------------
  // Internal data....
  //-------------------------------------------------------------------------
    
  UBool isLeapYear;

  //----------------------------------------------------------------------
  // Calendar framework
  //----------------------------------------------------------------------

 protected:
  virtual int32_t handleGetLimit(UCalendarDateFields field, ELimitType limitType) const;
  virtual int32_t handleGetMonthLength(int32_t extendedYear, int32_t month) const;
  virtual int32_t handleComputeMonthStart(int32_t eyear, int32_t month, UBool useMonth) const;
  virtual int32_t handleGetExtendedYear();
  virtual void handleComputeFields(int32_t julianDay, UErrorCode &status);
  virtual const UFieldResolutionTable* getFieldResolutionTable() const;



public:
  virtual void add(UCalendarDateFields field, int32_t amount, UErrorCode &status);
  virtual void add(EDateFields field, int32_t amount, UErrorCode &status);
  virtual void roll(UCalendarDateFields field, int32_t amount, UErrorCode &status);
  virtual void roll(EDateFields field, int32_t amount, UErrorCode &status);

  
  //----------------------------------------------------------------------
  // Internal methods & astronomical calculations
  //----------------------------------------------------------------------

private:

  static const UFieldResolutionTable CHINESE_DATE_PRECEDENCE[];

  static double daysToMillis(double days);
  static double millisToDays(double millis);
  virtual int32_t winterSolstice(int32_t gyear) const;
  virtual int32_t newMoonNear(double days, UBool after) const;
  virtual int32_t synodicMonthsBetween(int32_t day1, int32_t day2) const;
  virtual int32_t majorSolarTerm(int32_t days) const;
  virtual UBool hasNoMajorSolarTerm(int32_t newMoon) const;
  virtual UBool isLeapMonthBetween(int32_t newMoon1, int32_t newMoon2) const;
  virtual void computeChineseFields(int32_t days, int32_t gyear,
                 int32_t gmonth, UBool setAllFields);
  virtual int32_t newYear(int32_t gyear) const;
  virtual void offsetMonth(int32_t newMoon, int32_t dom, int32_t delta);


  // UObject stuff
 public: 
  /**
   * @return   The class ID for this object. All objects of a given class have the
   *           same class ID. Objects of other classes have different class IDs.
   * @internal
   */
  virtual UClassID getDynamicClassID(void) const;

  /**
   * Return the class ID for this class. This is useful only for comparing to a return
   * value from getDynamicClassID(). For example:
   *
   *      Base* polymorphic_pointer = createPolymorphicObject();
   *      if (polymorphic_pointer->getDynamicClassID() ==
   *          Derived::getStaticClassID()) ...
   *
   * @return   The class ID for all objects of this class.
   * @internal
   */
  U_I18N_API static UClassID U_EXPORT2 getStaticClassID(void);

  /**
   * return the calendar type, "chinese".
   *
   * @return calendar type
   * @internal
   */
  virtual const char * getType() const;


 protected:
  /**
   * (Overrides Calendar) Return true if the current date for this Calendar is in
   * Daylight Savings Time. Recognizes DST_OFFSET, if it is set.
   *
   * @param status Fill-in parameter which receives the status of this operation.
   * @return   True if the current date for this Calendar is in Daylight Savings Time,
   *           false, otherwise.
   * @internal
   */
  virtual UBool inDaylightTime(UErrorCode& status) const;


  /**
   * Returns TRUE because the Islamic Calendar does have a default century
   * @internal
   */
  virtual UBool haveDefaultCentury() const;

  /**
   * Returns the date of the start of the default century
   * @return start of century - in milliseconds since epoch, 1970
   * @internal
   */
  virtual UDate defaultCenturyStart() const;

  /**
   * Returns the year in which the default century begins
   * @internal
   */
  virtual int32_t defaultCenturyStartYear() const;

 private: // default century stuff.
  /**
   * The system maintains a static default century start date.  This is initialized
   * the first time it is used.  Before then, it is set to SYSTEM_DEFAULT_CENTURY to
   * indicate an uninitialized state.  Once the system default century date and year
   * are set, they do not change.
   */
  static UDate         fgSystemDefaultCenturyStart;

  /**
   * See documentation for systemDefaultCenturyStart.
   */
  static int32_t          fgSystemDefaultCenturyStartYear;

  /**
   * Default value that indicates the defaultCenturyStartYear is unitialized
   */
  static const int32_t    fgSystemDefaultCenturyYear;

  /**
   * start of default century, as a date
   */
  static const UDate        fgSystemDefaultCentury;

  /**
   * Returns the beginning date of the 100-year window that dates 
   * with 2-digit years are considered to fall within.
   */
  UDate         internalGetDefaultCenturyStart(void) const;

  /**
   * Returns the first year of the 100-year window that dates with 
   * 2-digit years are considered to fall within.
   */
  int32_t          internalGetDefaultCenturyStartYear(void) const;

  /**
   * Initializes the 100-year window that dates with 2-digit years
   * are considered to fall within so that its start date is 80 years
   * before the current time.
   */
  static void  initializeSystemDefaultCentury(void);

  ChineseCalendar(); // default constructor not implemented
};

U_NAMESPACE_END

#endif
#endif



