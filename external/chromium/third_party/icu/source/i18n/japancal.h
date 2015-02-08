
#ifndef JAPANCAL_H
#define JAPANCAL_H

#include "unicode/utypes.h"

#if !UCONFIG_NO_FORMATTING

#include "unicode/calendar.h"
#include "unicode/gregocal.h"

U_NAMESPACE_BEGIN

class JapaneseCalendar : public GregorianCalendar {
public:

    /**
     * Useful constants for JapaneseCalendar.  
     * @internal
     */
    U_I18N_API static uint32_t U_EXPORT2 getCurrentEra(void); // the current era

    /**
     * Constructs a JapaneseCalendar based on the current time in the default time zone
     * with the given locale.
     *
     * @param aLocale  The given locale.
     * @param success  Indicates the status of JapaneseCalendar object construction.
     *                 Returns U_ZERO_ERROR if constructed successfully.
     * @stable ICU 2.0
     */
    JapaneseCalendar(const Locale& aLocale, UErrorCode& success);


    /**
     * Destructor
     * @internal
     */
    virtual ~JapaneseCalendar();

    /**
     * Copy constructor
     * @param source    the object to be copied.
     * @internal
     */
    JapaneseCalendar(const JapaneseCalendar& source);

    /**
     * Default assignment operator
     * @param right    the object to be copied.
     * @internal
     */
    JapaneseCalendar& operator=(const JapaneseCalendar& right);

    /**
     * Create and return a polymorphic copy of this calendar.
     * @return    return a polymorphic copy of this calendar.
     * @internal
     */
    virtual Calendar* clone(void) const;

    /**
     * Return the extended year defined by the current fields.  In the 
     * Japanese calendar case, this is equal to the equivalent extended Gregorian year.
     * @internal
     */
    virtual int32_t handleGetExtendedYear();

    /**
     * Return the maximum value that this field could have, given the current date.
     * @internal
     */
    virtual int32_t getActualMaximum(UCalendarDateFields field, UErrorCode& status) const;


public:
    /**
     * Override Calendar Returns a unique class ID POLYMORPHICALLY. Pure virtual
     * override. This method is to implement a simple version of RTTI, since not all C++
     * compilers support genuine RTTI. Polymorphic operator==() and clone() methods call
     * this method.
     *
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
     * return the calendar type, "japanese".
     *
     * @return calendar type
     * @internal
     */
    virtual const char * getType() const;

    /**
     * @return FALSE - no default century in Japanese
     * @internal 
     */
    virtual UBool haveDefaultCentury() const;

    /**
     * Not used - no default century.
     * @internal
     */
    virtual UDate defaultCenturyStart() const;
    /**
     * Not used - no default century.
     * @internal
     */
    virtual int32_t defaultCenturyStartYear() const;

private:
    JapaneseCalendar(); // default constructor not implemented

protected:
    /** 
     * Calculate the era for internal computation
     * @internal
     */
    virtual int32_t internalGetEra() const;

    /**
     * Compute fields from the JD
     * @internal
     */
    virtual void handleComputeFields(int32_t julianDay, UErrorCode& status);

    /**
     * Calculate the limit for a specified type of limit and field
     * @internal
     */
    virtual int32_t handleGetLimit(UCalendarDateFields field, ELimitType limitType) const;

    /***
     * Called by computeJulianDay.  Returns the default month (0-based) for the year,
     * taking year and era into account.  Will return the first month of the given era, if 
     * the current year is an ascension year.
     * @param eyear the extended year
     * @internal
     */
    virtual int32_t getDefaultMonthInYear(int32_t eyear);

    /***
     * Called by computeJulianDay.  Returns the default day (1-based) for the month,
     * taking currently-set year and era into account.  Will return the first day of the given
     * era, if the current month is an ascension year and month.
     * @param eyear the extended year
     * @param mon the month in the year
     * @internal
     */
    virtual int32_t getDefaultDayInMonth(int32_t eyear, int32_t month);
};

U_NAMESPACE_END

#endif /* #if !UCONFIG_NO_FORMATTING */

#endif
//eof

