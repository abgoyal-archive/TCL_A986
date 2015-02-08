#ifndef OLSONTZ_H
#define OLSONTZ_H

#include "unicode/utypes.h"

#if !UCONFIG_NO_FORMATTING

#include "unicode/basictz.h"

struct UResourceBundle;

U_NAMESPACE_BEGIN

class SimpleTimeZone;

class OlsonTimeZone: public BasicTimeZone {
 public:
    /**
     * Construct from a resource bundle.
     * @param top the top-level zoneinfo resource bundle.  This is used
     * to lookup the rule that `res' may refer to, if there is one.
     * @param res the resource bundle of the zone to be constructed
     * @param ec input-output error code
     */
    OlsonTimeZone(const UResourceBundle* top,
                  const UResourceBundle* res, UErrorCode& ec);

    /**
     * Copy constructor
     */
    OlsonTimeZone(const OlsonTimeZone& other);

    /**
     * Destructor
     */
    virtual ~OlsonTimeZone();

    /**
     * Assignment operator
     */
    OlsonTimeZone& operator=(const OlsonTimeZone& other);

    /**
     * Returns true if the two TimeZone objects are equal.
     */
    virtual UBool operator==(const TimeZone& other) const;

    /**
     * TimeZone API.
     */
    virtual TimeZone* clone() const;

    /**
     * TimeZone API.
     */
    U_I18N_API static UClassID U_EXPORT2 getStaticClassID();

    /**
     * TimeZone API.
     */
    virtual UClassID getDynamicClassID() const;
    
    /**
     * TimeZone API.  Do not call this; prefer getOffset(UDate,...).
     */
    virtual int32_t getOffset(uint8_t era, int32_t year, int32_t month,
                              int32_t day, uint8_t dayOfWeek,
                              int32_t millis, UErrorCode& ec) const;

    /**
     * TimeZone API.  Do not call this; prefer getOffset(UDate,...).
     */
    virtual int32_t getOffset(uint8_t era, int32_t year, int32_t month,
                              int32_t day, uint8_t dayOfWeek,
                              int32_t millis, int32_t monthLength,
                              UErrorCode& ec) const;

    /**
     * TimeZone API.
     */
    virtual void getOffset(UDate date, UBool local, int32_t& rawOffset,
                   int32_t& dstOffset, UErrorCode& ec) const;

    /**
     * BasicTimeZone API.
     */
    virtual void getOffsetFromLocal(UDate date, int32_t nonExistingTimeOpt, int32_t duplicatedTimeOpt,
        int32_t& rawoff, int32_t& dstoff, UErrorCode& ec) /*const*/;

    /**
     * TimeZone API.  This method has no effect since objects of this
     * class are quasi-immutable (the base class allows the ID to be
     * changed).
     */
    virtual void setRawOffset(int32_t offsetMillis);

    /**
     * TimeZone API.  For a historical zone, the raw offset can change
     * over time, so this API is not useful.  In order to approximate
     * expected behavior, this method returns the raw offset for the
     * current moment in time.
     */
    virtual int32_t getRawOffset() const;

    /**
     * TimeZone API.  For a historical zone, whether DST is used or
     * not varies over time.  In order to approximate expected
     * behavior, this method returns TRUE if DST is observed at any
     * point in the current year.
     */
    virtual UBool useDaylightTime() const;

    /**
     * TimeZone API.
     */
    virtual UBool inDaylightTime(UDate date, UErrorCode& ec) const;

    /**
     * TimeZone API.
     */
    virtual int32_t getDSTSavings() const;

    /**
     * TimeZone API.  Also comare historic transitions.
     */
    virtual UBool hasSameRules(const TimeZone& other) const;

    /**
     * BasicTimeZone API.
     * Gets the first time zone transition after the base time.
     * @param base      The base time.
     * @param inclusive Whether the base time is inclusive or not.
     * @param result    Receives the first transition after the base time.
     * @return  TRUE if the transition is found.
     */
    virtual UBool getNextTransition(UDate base, UBool inclusive, TimeZoneTransition& result) /*const*/;

    /**
     * BasicTimeZone API.
     * Gets the most recent time zone transition before the base time.
     * @param base      The base time.
     * @param inclusive Whether the base time is inclusive or not.
     * @param result    Receives the most recent transition before the base time.
     * @return  TRUE if the transition is found.
     */
    virtual UBool getPreviousTransition(UDate base, UBool inclusive, TimeZoneTransition& result) /*const*/;

    /**
     * BasicTimeZone API.
     * Returns the number of <code>TimeZoneRule</code>s which represents time transitions,
     * for this time zone, that is, all <code>TimeZoneRule</code>s for this time zone except
     * <code>InitialTimeZoneRule</code>.  The return value range is 0 or any positive value.
     * @param status    Receives error status code.
     * @return The number of <code>TimeZoneRule</code>s representing time transitions.
     */
    virtual int32_t countTransitionRules(UErrorCode& status) /*const*/;

    /**
     * Gets the <code>InitialTimeZoneRule</code> and the set of <code>TimeZoneRule</code>
     * which represent time transitions for this time zone.  On successful return,
     * the argument initial points to non-NULL <code>InitialTimeZoneRule</code> and
     * the array trsrules is filled with 0 or multiple <code>TimeZoneRule</code>
     * instances up to the size specified by trscount.  The results are referencing the
     * rule instance held by this time zone instance.  Therefore, after this time zone
     * is destructed, they are no longer available.
     * @param initial       Receives the initial timezone rule
     * @param trsrules      Receives the timezone transition rules
     * @param trscount      On input, specify the size of the array 'transitions' receiving
     *                      the timezone transition rules.  On output, actual number of
     *                      rules filled in the array will be set.
     * @param status        Receives error status code.
     * @draft ICU 3.8
     */
    virtual void getTimeZoneRules(const InitialTimeZoneRule*& initial,
        const TimeZoneRule* trsrules[], int32_t& trscount, UErrorCode& status) /*const*/;

private:
    /**
     * Default constructor.  Creates a time zone with an empty ID and
     * a fixed GMT offset of zero.
     */
    OlsonTimeZone();

private:

    void constructEmpty();

    void getHistoricalOffset(UDate date, UBool local,
        int32_t NonExistingTimeOpt, int32_t DuplicatedTimeOpt,
        int32_t& rawoff, int32_t& dstoff) const;

    int16_t transitionCount() const;

    int64_t transitionTimeInSeconds(int16_t transIdx) const;
    double transitionTime(int16_t transIdx) const;

    /*
     * Following 3 methods return an offset at the given transition time index.
     * When the index is negative, return the initial offset.
     */
    int32_t zoneOffsetAt(int16_t transIdx) const;
    int32_t rawOffsetAt(int16_t transIdx) const;
    int32_t dstOffsetAt(int16_t transIdx) const;

    /*
     * Following methods return the initial offset.
     */
    int32_t initialRawOffset() const;
    int32_t initialDstOffset() const;

    /**
     * Number of transitions in each time range
     */
    int16_t transitionCountPre32;
    int16_t transitionCount32;
    int16_t transitionCountPost32;

    /**
     * Time of each transition in seconds from 1970 epoch before 32bit second range (<= 1900).
     * Each transition in this range is represented by a pair of int32_t.
     * Length is transitionCount int32_t's.  NULL if no transitions in this range.
     */
    const int32_t *transitionTimesPre32; // alias into res; do not delete

    /**
     * Time of each transition in seconds from 1970 epoch in 32bit second range.
     * Length is transitionCount int32_t's.  NULL if no transitions in this range.
     */
    const int32_t *transitionTimes32; // alias into res; do not delete

    /**
     * Time of each transition in seconds from 1970 epoch after 32bit second range (>= 2038).
     * Each transition in this range is represented by a pair of int32_t.
     * Length is transitionCount int32_t's.  NULL if no transitions in this range.
     */
    const int32_t *transitionTimesPost32; // alias into res; do not delete

    /**
     * Number of types, 1..255
     */
    int16_t typeCount;

    /**
     * Offset from GMT in seconds for each type.
     * Length is typeCount int32_t's.  At least one type (a pair of int32_t)
     * is required.
     */
    const int32_t *typeOffsets; // alias into res; do not delete

    /**
     * Type description data, consisting of transitionCount uint8_t
     * type indices (from 0..typeCount-1).
     * Length is transitionCount int16_t's.  NULL if no transitions.
     */
    const uint8_t *typeMapData; // alias into res; do not delete

    /**
     * A SimpleTimeZone that governs the behavior for date >= finalMillis.
     */
    SimpleTimeZone *finalZone; // owned, may be NULL

    /**
     * For date >= finalMillis, the finalZone will be used.
     */
    double finalStartMillis;

    /**
     * For year >= finalYear, the finalZone will be used.
     */
    int32_t finalStartYear;

    /* BasicTimeZone support */
    void clearTransitionRules(void);
    void deleteTransitionRules(void);
    void initTransitionRules(UErrorCode& status);

    InitialTimeZoneRule *initialRule;
    TimeZoneTransition  *firstTZTransition;
    int16_t             firstTZTransitionIdx;
    TimeZoneTransition  *firstFinalTZTransition;
    TimeArrayTimeZoneRule   **historicRules;
    int16_t             historicRuleCount;
    SimpleTimeZone      *finalZoneWithStartYear; // hack
    UBool               transitionRulesInitialized;
};

inline int16_t
OlsonTimeZone::transitionCount() const {
    return transitionCountPre32 + transitionCount32 + transitionCountPost32;
}

inline double
OlsonTimeZone::transitionTime(int16_t transIdx) const {
    return (double)transitionTimeInSeconds(transIdx) * U_MILLIS_PER_SECOND;
}

inline int32_t
OlsonTimeZone::zoneOffsetAt(int16_t transIdx) const {
    int16_t typeIdx = (transIdx >= 0 ? typeMapData[transIdx] : 0) << 1;
    return typeOffsets[typeIdx] + typeOffsets[typeIdx + 1];
}

inline int32_t
OlsonTimeZone::rawOffsetAt(int16_t transIdx) const {
    int16_t typeIdx = (transIdx >= 0 ? typeMapData[transIdx] : 0) << 1;
    return typeOffsets[typeIdx];
}

inline int32_t
OlsonTimeZone::dstOffsetAt(int16_t transIdx) const {
    int16_t typeIdx = (transIdx >= 0 ? typeMapData[transIdx] : 0) << 1;
    return typeOffsets[typeIdx + 1];
}

inline int32_t
OlsonTimeZone::initialRawOffset() const {
    return typeOffsets[0];
}

inline int32_t
OlsonTimeZone::initialDstOffset() const {
    return typeOffsets[1];
}

U_NAMESPACE_END

#endif // !UCONFIG_NO_FORMATTING
#endif // OLSONTZ_H

//eof
