
#include "chnsecal.h"

#if !UCONFIG_NO_FORMATTING

#include "umutex.h"
#include <float.h>
#include "gregoimp.h" // Math
#include "astro.h" // CalendarAstronomer
#include "uhash.h"
#include "ucln_in.h"

// Debugging
#ifdef U_DEBUG_CHNSECAL
# include <stdio.h>
# include <stdarg.h>
static void debug_chnsecal_loc(const char *f, int32_t l)
{
    fprintf(stderr, "%s:%d: ", f, l);
}

static void debug_chnsecal_msg(const char *pat, ...)
{
    va_list ap;
    va_start(ap, pat);
    vfprintf(stderr, pat, ap);
    fflush(stderr);
}
// must use double parens, i.e.:  U_DEBUG_CHNSECAL_MSG(("four is: %d",4));
#define U_DEBUG_CHNSECAL_MSG(x) {debug_chnsecal_loc(__FILE__,__LINE__);debug_chnsecal_msg x;}
#else
#define U_DEBUG_CHNSECAL_MSG(x)
#endif


// --- The cache --
static UMTX astroLock = 0;  // pod bay door lock
static U_NAMESPACE_QUALIFIER CalendarAstronomer *gChineseCalendarAstro = NULL;
static U_NAMESPACE_QUALIFIER CalendarCache *gChineseCalendarWinterSolsticeCache = NULL;
static U_NAMESPACE_QUALIFIER CalendarCache *gChineseCalendarNewYearCache = NULL;

static const int32_t CHINESE_EPOCH_YEAR = -2636; // Gregorian year

static const double CHINA_OFFSET = 8 * kOneHour;

static const int32_t SYNODIC_GAP = 25;


U_CDECL_BEGIN
static UBool calendar_chinese_cleanup(void) {
    if (gChineseCalendarAstro) {
        delete gChineseCalendarAstro;
        gChineseCalendarAstro = NULL;
    }
    if (gChineseCalendarWinterSolsticeCache) {
        delete gChineseCalendarWinterSolsticeCache;
        gChineseCalendarWinterSolsticeCache = NULL;
    }
    if (gChineseCalendarNewYearCache) {
        delete gChineseCalendarNewYearCache;
        gChineseCalendarNewYearCache = NULL;
    }
    umtx_destroy(&astroLock);
    return TRUE;
}
U_CDECL_END

U_NAMESPACE_BEGIN


// Implementation of the ChineseCalendar class


//-------------------------------------------------------------------------
// Constructors...
//-------------------------------------------------------------------------


Calendar* ChineseCalendar::clone() const {
    return new ChineseCalendar(*this);
}

ChineseCalendar::ChineseCalendar(const Locale& aLocale, UErrorCode& success)
:   Calendar(TimeZone::createDefault(), aLocale, success)
{
    isLeapYear = FALSE;
    setTimeInMillis(getNow(), success); // Call this again now that the vtable is set up properly.
}

ChineseCalendar::ChineseCalendar(const ChineseCalendar& other) : Calendar(other) {
    isLeapYear = other.isLeapYear;
}

ChineseCalendar::~ChineseCalendar()
{
}

const char *ChineseCalendar::getType() const { 
    return "chinese";
}

//-------------------------------------------------------------------------
// Minimum / Maximum access functions
//-------------------------------------------------------------------------


static const int32_t LIMITS[UCAL_FIELD_COUNT][4] = {
    // Minimum  Greatest     Least    Maximum
    //           Minimum   Maximum
    {        1,        1,    83333,    83333}, // ERA
    {        1,        1,       60,       60}, // YEAR
    {        0,        0,       11,       11}, // MONTH
    {        1,        1,       50,       55}, // WEEK_OF_YEAR
    {/*N/A*/-1,/*N/A*/-1,/*N/A*/-1,/*N/A*/-1}, // WEEK_OF_MONTH
    {        1,        1,       29,       30}, // DAY_OF_MONTH
    {        1,        1,      353,      385}, // DAY_OF_YEAR
    {/*N/A*/-1,/*N/A*/-1,/*N/A*/-1,/*N/A*/-1}, // DAY_OF_WEEK
    {       -1,       -1,        5,        5}, // DAY_OF_WEEK_IN_MONTH
    {/*N/A*/-1,/*N/A*/-1,/*N/A*/-1,/*N/A*/-1}, // AM_PM
    {/*N/A*/-1,/*N/A*/-1,/*N/A*/-1,/*N/A*/-1}, // HOUR
    {/*N/A*/-1,/*N/A*/-1,/*N/A*/-1,/*N/A*/-1}, // HOUR_OF_DAY
    {/*N/A*/-1,/*N/A*/-1,/*N/A*/-1,/*N/A*/-1}, // MINUTE
    {/*N/A*/-1,/*N/A*/-1,/*N/A*/-1,/*N/A*/-1}, // SECOND
    {/*N/A*/-1,/*N/A*/-1,/*N/A*/-1,/*N/A*/-1}, // MILLISECOND
    {/*N/A*/-1,/*N/A*/-1,/*N/A*/-1,/*N/A*/-1}, // ZONE_OFFSET
    {/*N/A*/-1,/*N/A*/-1,/*N/A*/-1,/*N/A*/-1}, // DST_OFFSET
    { -5000000, -5000000,  5000000,  5000000}, // YEAR_WOY
    {/*N/A*/-1,/*N/A*/-1,/*N/A*/-1,/*N/A*/-1}, // DOW_LOCAL
    { -5000000, -5000000,  5000000,  5000000}, // EXTENDED_YEAR
    {/*N/A*/-1,/*N/A*/-1,/*N/A*/-1,/*N/A*/-1}, // JULIAN_DAY
    {/*N/A*/-1,/*N/A*/-1,/*N/A*/-1,/*N/A*/-1}, // MILLISECONDS_IN_DAY
    {        0,        0,        1,        1}, // IS_LEAP_MONTH
};


int32_t ChineseCalendar::handleGetLimit(UCalendarDateFields field, ELimitType limitType) const {
    return LIMITS[field][limitType];
}


//----------------------------------------------------------------------
// Calendar framework
//----------------------------------------------------------------------

int32_t ChineseCalendar::handleGetExtendedYear() {
    int32_t year;
    if (newestStamp(UCAL_ERA, UCAL_YEAR, kUnset) <= fStamp[UCAL_EXTENDED_YEAR]) {
        year = internalGet(UCAL_EXTENDED_YEAR, 1); // Default to year 1
    } else {
        int32_t cycle = internalGet(UCAL_ERA, 1) - 1; // 0-based cycle
        year = cycle * 60 + internalGet(UCAL_YEAR, 1);
    }
    return year;
}

int32_t ChineseCalendar::handleGetMonthLength(int32_t extendedYear, int32_t month) const {
    int32_t thisStart = handleComputeMonthStart(extendedYear, month, TRUE) -
        kEpochStartAsJulianDay + 1; // Julian day -> local days
    int32_t nextStart = newMoonNear(thisStart + SYNODIC_GAP, TRUE);
    return nextStart - thisStart;
}

void ChineseCalendar::handleComputeFields(int32_t julianDay, UErrorCode &/*status*/) {

    computeChineseFields(julianDay - kEpochStartAsJulianDay, // local days
                         getGregorianYear(), getGregorianMonth(),
                         TRUE); // set all fields
}

const UFieldResolutionTable ChineseCalendar::CHINESE_DATE_PRECEDENCE[] =
{
    {
        { UCAL_DAY_OF_MONTH, kResolveSTOP },
        { UCAL_WEEK_OF_YEAR, UCAL_DAY_OF_WEEK, kResolveSTOP },
        { UCAL_WEEK_OF_MONTH, UCAL_DAY_OF_WEEK, kResolveSTOP },
        { UCAL_DAY_OF_WEEK_IN_MONTH, UCAL_DAY_OF_WEEK, kResolveSTOP },
        { UCAL_WEEK_OF_YEAR, UCAL_DOW_LOCAL, kResolveSTOP },
        { UCAL_WEEK_OF_MONTH, UCAL_DOW_LOCAL, kResolveSTOP },
        { UCAL_DAY_OF_WEEK_IN_MONTH, UCAL_DOW_LOCAL, kResolveSTOP },
        { UCAL_DAY_OF_YEAR, kResolveSTOP },
        { kResolveRemap | UCAL_DAY_OF_MONTH, UCAL_IS_LEAP_MONTH, kResolveSTOP },
        { kResolveSTOP }
    },
    {
        { UCAL_WEEK_OF_YEAR, kResolveSTOP },
        { UCAL_WEEK_OF_MONTH, kResolveSTOP },
        { UCAL_DAY_OF_WEEK_IN_MONTH, kResolveSTOP },
        { kResolveRemap | UCAL_DAY_OF_WEEK_IN_MONTH, UCAL_DAY_OF_WEEK, kResolveSTOP },
        { kResolveRemap | UCAL_DAY_OF_WEEK_IN_MONTH, UCAL_DOW_LOCAL, kResolveSTOP },
        { kResolveSTOP }
    },
    {{kResolveSTOP}}
};

const UFieldResolutionTable* ChineseCalendar::getFieldResolutionTable() const {
    return CHINESE_DATE_PRECEDENCE;
}

int32_t ChineseCalendar::handleComputeMonthStart(int32_t eyear, int32_t month, UBool useMonth) const {

    ChineseCalendar *nonConstThis = (ChineseCalendar*)this; // cast away const

    // If the month is out of range, adjust it into range, and
    // modify the extended year value accordingly.
    if (month < 0 || month > 11) {
        double m = month;
        eyear += (int32_t)ClockMath::floorDivide(m, 12.0, m);
        month = (int32_t)m;
    }

    int32_t gyear = eyear + CHINESE_EPOCH_YEAR - 1; // Gregorian year
    int32_t theNewYear = newYear(gyear);
    int32_t newMoon = newMoonNear(theNewYear + month * 29, TRUE);
    
    int32_t julianDay = newMoon + kEpochStartAsJulianDay;

    // Save fields for later restoration
    int32_t saveMonth = internalGet(UCAL_MONTH);
    int32_t saveIsLeapMonth = internalGet(UCAL_IS_LEAP_MONTH);

    // Ignore IS_LEAP_MONTH field if useMonth is false
    int32_t isLeapMonth = useMonth ? saveIsLeapMonth : 0;

    UErrorCode status = U_ZERO_ERROR;
    nonConstThis->computeGregorianFields(julianDay, status);
    if (U_FAILURE(status))
        return 0;
    
    // This will modify the MONTH and IS_LEAP_MONTH fields (only)
    nonConstThis->computeChineseFields(newMoon, getGregorianYear(),
                         getGregorianMonth(), FALSE);        

    if (month != internalGet(UCAL_MONTH) ||
        isLeapMonth != internalGet(UCAL_IS_LEAP_MONTH)) {
        newMoon = newMoonNear(newMoon + SYNODIC_GAP, TRUE);
        julianDay = newMoon + kEpochStartAsJulianDay;
    }

    nonConstThis->internalSet(UCAL_MONTH, saveMonth);
    nonConstThis->internalSet(UCAL_IS_LEAP_MONTH, saveIsLeapMonth);

    return julianDay - 1;
}


void ChineseCalendar::add(UCalendarDateFields field, int32_t amount, UErrorCode& status) {
    switch (field) {
    case UCAL_MONTH:
        if (amount != 0) {
            int32_t dom = get(UCAL_DAY_OF_MONTH, status);
            if (U_FAILURE(status)) break;
            int32_t day = get(UCAL_JULIAN_DAY, status) - kEpochStartAsJulianDay; // Get local day
            if (U_FAILURE(status)) break;
            int32_t moon = day - dom + 1; // New moon 
            offsetMonth(moon, dom, amount);
        }
        break;
    default:
        Calendar::add(field, amount, status);
        break;
    }
}

void ChineseCalendar::add(EDateFields field, int32_t amount, UErrorCode& status) {
    add((UCalendarDateFields)field, amount, status);
}

void ChineseCalendar::roll(UCalendarDateFields field, int32_t amount, UErrorCode& status) {
    switch (field) {
    case UCAL_MONTH:
        if (amount != 0) {
            int32_t dom = get(UCAL_DAY_OF_MONTH, status);
            if (U_FAILURE(status)) break;
            int32_t day = get(UCAL_JULIAN_DAY, status) - kEpochStartAsJulianDay; // Get local day
            if (U_FAILURE(status)) break;
            int32_t moon = day - dom + 1; // New moon (start of this month)

            // Note throughout the following:  Months 12 and 1 are never
            // followed by a leap month (D&R p. 185).

            // Compute the adjusted month number m.  This is zero-based
            // value from 0..11 in a non-leap year, and from 0..12 in a
            // leap year.
            int32_t m = get(UCAL_MONTH, status); // 0-based month
            if (U_FAILURE(status)) break;
            if (isLeapYear) { // (member variable)
                if (get(UCAL_IS_LEAP_MONTH, status) == 1) {
                    ++m;
                } else {
                    // Check for a prior leap month.  (In the
                    // following, month 0 is the first month of the
                    // year.)  Month 0 is never followed by a leap
                    // month, and we know month m is not a leap month.
                    // moon1 will be the start of month 0 if there is
                    // no leap month between month 0 and month m;
                    // otherwise it will be the start of month 1.
                    int moon1 = moon -
                        (int) (CalendarAstronomer::SYNODIC_MONTH * (m - 0.5));
                    moon1 = newMoonNear(moon1, TRUE);
                    if (isLeapMonthBetween(moon1, moon)) {
                        ++m;
                    }
                }
                if (U_FAILURE(status)) break;
            }

            // Now do the standard roll computation on m, with the
            // allowed range of 0..n-1, where n is 12 or 13.
            int32_t n = isLeapYear ? 13 : 12; // Months in this year
            int32_t newM = (m + amount) % n;
            if (newM < 0) {
                newM += n;
            }

            if (newM != m) {
                offsetMonth(moon, dom, newM - m);
            }
        }
        break;
    default:
        Calendar::roll(field, amount, status);
        break;
    }
}

void ChineseCalendar::roll(EDateFields field, int32_t amount, UErrorCode& status) {
    roll((UCalendarDateFields)field, amount, status);
}


//------------------------------------------------------------------
// Support methods and constants
//------------------------------------------------------------------

double ChineseCalendar::daysToMillis(double days) {
    return (days * kOneDay) - CHINA_OFFSET;
}

double ChineseCalendar::millisToDays(double millis) {
    return ClockMath::floorDivide(millis + CHINA_OFFSET, kOneDay);
}

//------------------------------------------------------------------
// Astronomical computations
//------------------------------------------------------------------


int32_t ChineseCalendar::winterSolstice(int32_t gyear) const {

    UErrorCode status = U_ZERO_ERROR;
    int32_t cacheValue = CalendarCache::get(&gChineseCalendarWinterSolsticeCache, gyear, status);

    if (cacheValue == 0) {
        // In books December 15 is used, but it fails for some years
        // using our algorithms, e.g.: 1298 1391 1492 1553 1560.  That
        // is, winterSolstice(1298) starts search at Dec 14 08:00:00
        // PST 1298 with a final result of Dec 14 10:31:59 PST 1299.
        double ms = daysToMillis(Grego::fieldsToDay(gyear, UCAL_DECEMBER, 1));

        umtx_lock(&astroLock);
        if(gChineseCalendarAstro == NULL) {
            gChineseCalendarAstro = new CalendarAstronomer();
            ucln_i18n_registerCleanup(UCLN_I18N_CHINESE_CALENDAR, calendar_chinese_cleanup);
        }
        gChineseCalendarAstro->setTime(ms);
        UDate solarLong = gChineseCalendarAstro->getSunTime(CalendarAstronomer::WINTER_SOLSTICE(), TRUE);
        umtx_unlock(&astroLock);

        // Winter solstice is 270 degrees solar longitude aka Dongzhi
        cacheValue = (int32_t)millisToDays(solarLong);
        CalendarCache::put(&gChineseCalendarWinterSolsticeCache, gyear, cacheValue, status);
    }
    if(U_FAILURE(status)) {
        cacheValue = 0;
    }
    return cacheValue;
}

int32_t ChineseCalendar::newMoonNear(double days, UBool after) const {
    
    umtx_lock(&astroLock);
    if(gChineseCalendarAstro == NULL) {
        gChineseCalendarAstro = new CalendarAstronomer();
        ucln_i18n_registerCleanup(UCLN_I18N_CHINESE_CALENDAR, calendar_chinese_cleanup);
    }
    gChineseCalendarAstro->setTime(daysToMillis(days));
    UDate newMoon = gChineseCalendarAstro->getMoonTime(CalendarAstronomer::NEW_MOON(), after);
    umtx_unlock(&astroLock);
    
    return (int32_t) millisToDays(newMoon);
}

int32_t ChineseCalendar::synodicMonthsBetween(int32_t day1, int32_t day2) const {
    double roundme = ((day2 - day1) / CalendarAstronomer::SYNODIC_MONTH);
    return (int32_t) (roundme + (roundme >= 0 ? .5 : -.5));
}

int32_t ChineseCalendar::majorSolarTerm(int32_t days) const {
    
    umtx_lock(&astroLock);
    if(gChineseCalendarAstro == NULL) {
        gChineseCalendarAstro = new CalendarAstronomer();
        ucln_i18n_registerCleanup(UCLN_I18N_CHINESE_CALENDAR, calendar_chinese_cleanup);
    }
    gChineseCalendarAstro->setTime(daysToMillis(days));
    UDate solarLongitude = gChineseCalendarAstro->getSunLongitude();
    umtx_unlock(&astroLock);

    // Compute (floor(solarLongitude / (pi/6)) + 2) % 12
    int32_t term = ( ((int32_t)(6 * solarLongitude / CalendarAstronomer::PI)) + 2 ) % 12;
    if (term < 1) {
        term += 12;
    }
    return term;
}

UBool ChineseCalendar::hasNoMajorSolarTerm(int32_t newMoon) const {
    return majorSolarTerm(newMoon) ==
        majorSolarTerm(newMoonNear(newMoon + SYNODIC_GAP, TRUE));
}


//------------------------------------------------------------------
// Time to fields
//------------------------------------------------------------------

UBool ChineseCalendar::isLeapMonthBetween(int32_t newMoon1, int32_t newMoon2) const {

#ifdef U_DEBUG_CHNSECAL
    // This is only needed to debug the timeOfAngle divergence bug.
    // Remove this later. Liu 11/9/00
    if (synodicMonthsBetween(newMoon1, newMoon2) >= 50) {
        U_DEBUG_CHNSECAL_MSG((
            "isLeapMonthBetween(%d, %d): Invalid parameters", newMoon1, newMoon2
            ));
    }
#endif

    return (newMoon2 >= newMoon1) &&
        (isLeapMonthBetween(newMoon1, newMoonNear(newMoon2 - SYNODIC_GAP, FALSE)) ||
         hasNoMajorSolarTerm(newMoon2));
}

void ChineseCalendar::computeChineseFields(int32_t days, int32_t gyear, int32_t gmonth,
                                  UBool setAllFields) {

    // Find the winter solstices before and after the target date.
    // These define the boundaries of this Chinese year, specifically,
    // the position of month 11, which always contains the solstice.
    // We want solsticeBefore <= date < solsticeAfter.
    int32_t solsticeBefore;
    int32_t solsticeAfter = winterSolstice(gyear);
    if (days < solsticeAfter) {
        solsticeBefore = winterSolstice(gyear - 1);
    } else {
        solsticeBefore = solsticeAfter;
        solsticeAfter = winterSolstice(gyear + 1);
    }

    // Find the start of the month after month 11.  This will be either
    // the prior month 12 or leap month 11 (very rare).  Also find the
    // start of the following month 11.
    int32_t firstMoon = newMoonNear(solsticeBefore + 1, TRUE);
    int32_t lastMoon = newMoonNear(solsticeAfter + 1, FALSE);
    int32_t thisMoon = newMoonNear(days + 1, FALSE); // Start of this month
    // Note: isLeapYear is a member variable
    isLeapYear = synodicMonthsBetween(firstMoon, lastMoon) == 12;

    int32_t month = synodicMonthsBetween(firstMoon, thisMoon);
    if (isLeapYear && isLeapMonthBetween(firstMoon, thisMoon)) {
        month--;
    }
    if (month < 1) {
        month += 12;
    }

    UBool isLeapMonth = isLeapYear &&
        hasNoMajorSolarTerm(thisMoon) &&
        !isLeapMonthBetween(firstMoon, newMoonNear(thisMoon - SYNODIC_GAP, FALSE));

    internalSet(UCAL_MONTH, month-1); // Convert from 1-based to 0-based
    internalSet(UCAL_IS_LEAP_MONTH, isLeapMonth?1:0);

    if (setAllFields) {

        int32_t year = gyear - CHINESE_EPOCH_YEAR;
        if (month < 11 ||
            gmonth >= UCAL_JULY) {
            year++;
        }
        int32_t dayOfMonth = days - thisMoon + 1;

        internalSet(UCAL_EXTENDED_YEAR, year);

        // 0->0,60  1->1,1  60->1,60  61->2,1  etc.
        int32_t yearOfCycle;
        int32_t cycle = ClockMath::floorDivide(year - 1, 60, yearOfCycle);
        internalSet(UCAL_ERA, cycle + 1);
        internalSet(UCAL_YEAR, yearOfCycle + 1);

        internalSet(UCAL_DAY_OF_MONTH, dayOfMonth);

        // Days will be before the first new year we compute if this
        // date is in month 11, leap 11, 12.  There is never a leap 12.
        // New year computations are cached so this should be cheap in
        // the long run.
        int32_t theNewYear = newYear(gyear);
        if (days < theNewYear) {
            theNewYear = newYear(gyear-1);
        }
        internalSet(UCAL_DAY_OF_YEAR, days - theNewYear + 1);
    }
}


//------------------------------------------------------------------
// Fields to time
//------------------------------------------------------------------

int32_t ChineseCalendar::newYear(int32_t gyear) const {
    UErrorCode status = U_ZERO_ERROR;
    int32_t cacheValue = CalendarCache::get(&gChineseCalendarNewYearCache, gyear, status);

    if (cacheValue == 0) {

        int32_t solsticeBefore= winterSolstice(gyear - 1);
        int32_t solsticeAfter = winterSolstice(gyear);
        int32_t newMoon1 = newMoonNear(solsticeBefore + 1, TRUE);
        int32_t newMoon2 = newMoonNear(newMoon1 + SYNODIC_GAP, TRUE);
        int32_t newMoon11 = newMoonNear(solsticeAfter + 1, FALSE);
        
        if (synodicMonthsBetween(newMoon1, newMoon11) == 12 &&
            (hasNoMajorSolarTerm(newMoon1) || hasNoMajorSolarTerm(newMoon2))) {
            cacheValue = newMoonNear(newMoon2 + SYNODIC_GAP, TRUE);
        } else {
            cacheValue = newMoon2;
        }

        CalendarCache::put(&gChineseCalendarNewYearCache, gyear, cacheValue, status);
    }
    if(U_FAILURE(status)) {
        cacheValue = 0;
    }
    return cacheValue;
}

void ChineseCalendar::offsetMonth(int32_t newMoon, int32_t dom, int32_t delta) {
    UErrorCode status = U_ZERO_ERROR;

    // Move to the middle of the month before our target month.
    newMoon += (int32_t) (CalendarAstronomer::SYNODIC_MONTH * (delta - 0.5));

    // Search forward to the target month's new moon
    newMoon = newMoonNear(newMoon, TRUE);

    // Find the target dom
    int32_t jd = newMoon + kEpochStartAsJulianDay - 1 + dom;

    // Pin the dom.  In this calendar all months are 29 or 30 days
    // so pinning just means handling dom 30.
    if (dom > 29) {
        set(UCAL_JULIAN_DAY, jd-1);
        // TODO Fix this.  We really shouldn't ever have to
        // explicitly call complete().  This is either a bug in
        // this method, in ChineseCalendar, or in
        // Calendar.getActualMaximum().  I suspect the last.
        complete(status);
        if (U_FAILURE(status)) return;
        if (getActualMaximum(UCAL_DAY_OF_MONTH, status) >= dom) {
            if (U_FAILURE(status)) return;
            set(UCAL_JULIAN_DAY, jd);
        }
    } else {
        set(UCAL_JULIAN_DAY, jd);
    }
}


UBool
ChineseCalendar::inDaylightTime(UErrorCode& status) const
{
    // copied from GregorianCalendar
    if (U_FAILURE(status) || !getTimeZone().useDaylightTime()) 
        return FALSE;

    // Force an update of the state of the Calendar.
    ((ChineseCalendar*)this)->complete(status); // cast away const

    return (UBool)(U_SUCCESS(status) ? (internalGet(UCAL_DST_OFFSET) != 0) : FALSE);
}

// default century
const UDate     ChineseCalendar::fgSystemDefaultCentury        = DBL_MIN;
const int32_t   ChineseCalendar::fgSystemDefaultCenturyYear    = -1;

UDate           ChineseCalendar::fgSystemDefaultCenturyStart       = DBL_MIN;
int32_t         ChineseCalendar::fgSystemDefaultCenturyStartYear   = -1;


UBool ChineseCalendar::haveDefaultCentury() const
{
    return TRUE;
}

UDate ChineseCalendar::defaultCenturyStart() const
{
    return internalGetDefaultCenturyStart();
}

int32_t ChineseCalendar::defaultCenturyStartYear() const
{
    return internalGetDefaultCenturyStartYear();
}

UDate
ChineseCalendar::internalGetDefaultCenturyStart() const
{
    // lazy-evaluate systemDefaultCenturyStart
    UBool needsUpdate;
    UMTX_CHECK(NULL, (fgSystemDefaultCenturyStart == fgSystemDefaultCentury), needsUpdate);

    if (needsUpdate) {
        initializeSystemDefaultCentury();
    }

    // use defaultCenturyStart unless it's the flag value;
    // then use systemDefaultCenturyStart

    return fgSystemDefaultCenturyStart;
}

int32_t
ChineseCalendar::internalGetDefaultCenturyStartYear() const
{
    // lazy-evaluate systemDefaultCenturyStartYear
    UBool needsUpdate;
    UMTX_CHECK(NULL, (fgSystemDefaultCenturyStart == fgSystemDefaultCentury), needsUpdate);

    if (needsUpdate) {
        initializeSystemDefaultCentury();
    }

    // use defaultCenturyStart unless it's the flag value;
    // then use systemDefaultCenturyStartYear

    return    fgSystemDefaultCenturyStartYear;
}

void
ChineseCalendar::initializeSystemDefaultCentury()
{
    // initialize systemDefaultCentury and systemDefaultCenturyYear based
    // on the current time.  They'll be set to 80 years before
    // the current time.
    UErrorCode status = U_ZERO_ERROR;
    ChineseCalendar calendar(Locale("@calendar=chinese"),status);
    if (U_SUCCESS(status))
    {
        calendar.setTime(Calendar::getNow(), status);
        calendar.add(UCAL_YEAR, -80, status);
        UDate    newStart =  calendar.getTime(status);
        int32_t  newYear  =  calendar.get(UCAL_YEAR, status);
        umtx_lock(NULL);
        if (fgSystemDefaultCenturyStart == fgSystemDefaultCentury)
        {
            fgSystemDefaultCenturyStartYear = newYear;
            fgSystemDefaultCenturyStart = newStart;
        }
        umtx_unlock(NULL);
    }
    // We have no recourse upon failure unless we want to propagate the failure
    // out.
}

UOBJECT_DEFINE_RTTI_IMPLEMENTATION(ChineseCalendar)

U_NAMESPACE_END

#endif

