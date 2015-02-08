
#ifndef __DTITVINF_H__
#define __DTITVINF_H__

#include "unicode/utypes.h"


#if !UCONFIG_NO_FORMATTING

#include "unicode/udat.h"
#include "unicode/locid.h"
#include "unicode/ucal.h"
#include "unicode/dtptngen.h"
//#include "dtitv_impl.h"

 
union UHashTok;


U_NAMESPACE_BEGIN

U_CDECL_BEGIN 

UBool U_CALLCONV dtitvinfHashTableValueComparator(UHashTok val1, UHashTok val2) ;

U_CDECL_END 



class U_I18N_API DateIntervalInfo : public UObject {
public:
    /**
     * Default constructor.
     * It does not initialize any interval patterns except
     * that it initialize default fall-back pattern as "{0} - {1}",
     * which can be reset by setFallbackIntervalPattern().
     * It should be followed by setFallbackIntervalPattern() and 
     * setIntervalPattern(), 
     * and is recommended to be used only for power users who
     * wants to create their own interval patterns and use them to create
     * date interval formatter.
     * @param status   output param set to success/failure code on exit
     * @internal ICU 4.0
     */
    DateIntervalInfo(UErrorCode& status);


    /** 
     * Construct DateIntervalInfo for the given locale,
     * @param locale  the interval patterns are loaded from the Gregorian 
     *                calendar data in this locale.
     * @param status  output param set to success/failure code on exit
     * @stable ICU 4.0
     */
    DateIntervalInfo(const Locale& locale, UErrorCode& status);


    /**
     * Copy constructor.
     * @stable ICU 4.0
     */
    DateIntervalInfo(const DateIntervalInfo&);

    /**
     * Assignment operator
     * @stable ICU 4.0
     */
    DateIntervalInfo& operator=(const DateIntervalInfo&);

    /**
     * Clone this object polymorphically.
     * The caller owns the result and should delete it when done.
     * @return   a copy of the object
     * @stable ICU 4.0
     */
    virtual DateIntervalInfo* clone(void) const;

    /**
     * Destructor.
     * It is virtual to be safe, but it is not designed to be subclassed.
     * @stable ICU 4.0
     */
    virtual ~DateIntervalInfo();


    /**
     * Return true if another object is semantically equal to this one.
     *
     * @param other    the DateIntervalInfo object to be compared with.
     * @return         true if other is semantically equal to this.
     * @stable ICU 4.0
     */
    virtual UBool operator==(const DateIntervalInfo& other) const;

    /**
     * Return true if another object is semantically unequal to this one.
     *
     * @param other    the DateIntervalInfo object to be compared with.
     * @return         true if other is semantically unequal to this.
     * @stable ICU 4.0
     */
    UBool operator!=(const DateIntervalInfo& other) const;



    /** 
     * Provides a way for client to build interval patterns.
     * User could construct DateIntervalInfo by providing a list of skeletons
     * and their patterns.
     * <P>
     * For example:
     * <pre>
     * UErrorCode status = U_ZERO_ERROR;
     * DateIntervalInfo dIntervalInfo = new DateIntervalInfo();
     * dIntervalInfo->setFallbackIntervalPattern("{0} ~ {1}");
     * dIntervalInfo->setIntervalPattern("yMd", UCAL_YEAR, "'from' yyyy-M-d 'to' yyyy-M-d", status); 
     * dIntervalInfo->setIntervalPattern("yMMMd", UCAL_MONTH, "'from' yyyy MMM d 'to' MMM d", status);
     * dIntervalInfo->setIntervalPattern("yMMMd", UCAL_DAY, "yyyy MMM d-d", status, status);
     * </pre>
     *
     * Restriction: 
     * Currently, users can only set interval patterns when the following 
     * calendar fields are different: ERA, YEAR, MONTH, DATE,  DAY_OF_MONTH, 
     * DAY_OF_WEEK, AM_PM,  HOUR, HOUR_OF_DAY, and MINUTE.
     * Interval patterns when other calendar fields are different are 
     * not supported.
     *
     * @param skeleton         the skeleton on which interval pattern based
     * @param lrgDiffCalUnit   the largest different calendar unit.
     * @param intervalPattern  the interval pattern on the largest different
     *                         calendar unit.
     *                         For example, if lrgDiffCalUnit is 
     *                         "year", the interval pattern for en_US when year
     *                         is different could be "'from' yyyy 'to' yyyy".
     * @param status           output param set to success/failure code on exit
     * @stable ICU 4.0
     */
    void setIntervalPattern(const UnicodeString& skeleton, 
                            UCalendarDateFields lrgDiffCalUnit, 
                            const UnicodeString& intervalPattern,
                            UErrorCode& status);

    /**
     * Get the interval pattern given skeleton and 
     * the largest different calendar field.
     * @param skeleton   the skeleton
     * @param field      the largest different calendar field
     * @param result     output param to receive the pattern
     * @param status     output param set to success/failure code on exit
     * @return a reference to 'result'
     * @stable ICU 4.0 
     */
    UnicodeString& getIntervalPattern(const UnicodeString& skeleton,
                                      UCalendarDateFields field,
                                      UnicodeString& result,
                                      UErrorCode& status) const; 

    /**
     * Get the fallback interval pattern.
     * @param  result   output param to receive the pattern
     * @return a reference to 'result'
     * @stable ICU 4.0 
     */
    UnicodeString& getFallbackIntervalPattern(UnicodeString& result) const;


    /**
     * Re-set the fallback interval pattern.
     *
     * In construction, default fallback pattern is set as "{0} - {1}".
     * And constructor taking locale as parameter will set the
     * fallback pattern as what defined in the locale resource file.
     *
     * This method provides a way for user to replace the fallback pattern.
     *
     * @param fallbackPattern  fall-back interval pattern.
     * @param status           output param set to success/failure code on exit
     * @stable ICU 4.0 
     */
    void setFallbackIntervalPattern(const UnicodeString& fallbackPattern,
                                    UErrorCode& status);


    /** Get default order -- whether the first date in pattern is later date
                             or not.
     * return default date ordering in interval pattern. TRUE if the first date
     *        in pattern is later date, FALSE otherwise.
     * @stable ICU 4.0 
     */
    UBool getDefaultOrder() const;


    /**
     * ICU "poor man's RTTI", returns a UClassID for the actual class.
     *
     * @stable ICU 4.0
     */
    virtual UClassID getDynamicClassID() const;

    /**
     * ICU "poor man's RTTI", returns a UClassID for this class.
     *
     * @stable ICU 4.0
     */
    static UClassID U_EXPORT2 getStaticClassID();


private:
    /**
     * DateIntervalFormat will need access to
     * getBestSkeleton(), parseSkeleton(), enum IntervalPatternIndex,
     * and calendarFieldToPatternIndex().
     *
     * Instead of making above public,
     * make DateIntervalFormat a friend of DateIntervalInfo.
     */
    friend class DateIntervalFormat;

    friend UBool U_CALLCONV dtitvinfHashTableValueComparator(UHashTok val1, UHashTok val2) ;

    /**
     * Following is for saving the interval patterns.
     * We only support interval patterns on
     * ERA, YEAR, MONTH, DAY, AM_PM, HOUR, and MINUTE
     */
    enum IntervalPatternIndex
    {
        kIPI_ERA,
        kIPI_YEAR,
        kIPI_MONTH,
        kIPI_DATE,
        kIPI_AM_PM,
        kIPI_HOUR,
        kIPI_MINUTE,
        kIPI_MAX_INDEX
    };

    /** 
     * Initialize the DateIntervalInfo from locale
     * @param locale   the given locale.
     * @param status   output param set to success/failure code on exit
     * @internal ICU 4.0 
     */
    void initializeData(const Locale& locale, UErrorCode& status);


    /* Set Interval pattern.
     *
     * It sets interval pattern into the hash map.
     *
     * @param skeleton         skeleton on which the interval pattern based
     * @param lrgDiffCalUnit   the largest different calendar unit.
     * @param intervalPattern  the interval pattern on the largest different
     *                         calendar unit.
     * @param status           output param set to success/failure code on exit
     * @internal ICU 4.0
     */
    void setIntervalPatternInternally(const UnicodeString& skeleton,
                                      UCalendarDateFields lrgDiffCalUnit,
                                      const UnicodeString& intervalPattern,
                                      UErrorCode& status); 


    /**given an input skeleton, get the best match skeleton 
     * which has pre-defined interval pattern in resource file.
     * Also return the difference between the input skeleton
     * and the best match skeleton.
     *
     * TODO (xji): set field weight or
     *             isolate the funtionality in DateTimePatternGenerator
     * @param  skeleton               input skeleton
     * @param  bestMatchDistanceInfo  the difference between input skeleton
     *                                and best match skeleton.
     *         0, if there is exact match for input skeleton
     *         1, if there is only field width difference between 
     *            the best match and the input skeleton
     *         2, the only field difference is 'v' and 'z'
     *        -1, if there is calendar field difference between
     *            the best match and the input skeleton
     * @return                        best match skeleton
     * @internal ICU 4.0
     */
    const UnicodeString* getBestSkeleton(const UnicodeString& skeleton,
                                         int8_t& bestMatchDistanceInfo) const;


    /**
     * Parse skeleton, save each field's width.
     * It is used for looking for best match skeleton,
     * and adjust pattern field width.
     * @param skeleton            skeleton to be parsed
     * @param skeletonFieldWidth  parsed skeleton field width
     * @internal ICU 4.0
     */
    static void U_EXPORT2 parseSkeleton(const UnicodeString& skeleton, 
                                        int32_t* skeletonFieldWidth);


    /**
     * Check whether one field width is numeric while the other is string.
     *
     * TODO (xji): make it general
     *
     * @param fieldWidth          one field width
     * @param anotherFieldWidth   another field width
     * @param patternLetter       pattern letter char
     * @return true if one field width is numeric and the other is string,
     *         false otherwise.
     * @internal ICU 4.0
     */
    static UBool U_EXPORT2 stringNumeric(int32_t fieldWidth,
                                         int32_t anotherFieldWidth,
                                         char patternLetter);


    /** 
     * Convert calendar field to the interval pattern index in 
     * hash table.
     *
     * Since we only support the following calendar fields: 
     * ERA, YEAR, MONTH, DATE,  DAY_OF_MONTH, DAY_OF_WEEK, 
     * AM_PM,  HOUR, HOUR_OF_DAY, and MINUTE,
     * We reserve only 4 interval patterns for a skeleton.
     *
     * @param field    calendar field
     * @param status   output param set to success/failure code on exit
     * @return  interval pattern index in hash table
     * @internal ICU 4.0
     */
    static IntervalPatternIndex U_EXPORT2 calendarFieldToIntervalIndex(
                                                      UCalendarDateFields field,
                                                      UErrorCode& status);


    /**
     * delete hash table (of type fIntervalPatterns).
     *
     * @param hTable  hash table to be deleted
     * @internal ICU 4.0
     */
    void deleteHash(Hashtable* hTable);


    /**
     * initialize hash table (of type fIntervalPatterns).
     *
     * @param status   output param set to success/failure code on exit
     * @return         hash table initialized
     * @internal ICU 4.0
     */
    Hashtable* initHash(UErrorCode& status);



    /**
     * copy hash table (of type fIntervalPatterns).
     *
     * @param source   the source to copy from
     * @param target   the target to copy to
     * @param status   output param set to success/failure code on exit
     * @internal ICU 4.0
     */
    void copyHash(const Hashtable* source, Hashtable* target, UErrorCode& status);


    // data members
    // fallback interval pattern 
    UnicodeString fFallbackIntervalPattern;
    // default order
    UBool fFirstDateInPtnIsLaterDate;

    // HashMap<UnicodeString, UnicodeString[kIPI_MAX_INDEX]>
    // HashMap( skeleton, pattern[largest_different_field] )
    Hashtable* fIntervalPatterns;

};// end class DateIntervalInfo


inline UBool
DateIntervalInfo::operator!=(const DateIntervalInfo& other) const {
    return !operator==(other);
}


U_NAMESPACE_END

#endif

#endif

