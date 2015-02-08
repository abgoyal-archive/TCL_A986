
#ifndef UTMSCALE_H
#define UTMSCALE_H

#include "unicode/utypes.h"

#if !UCONFIG_NO_FORMATTING


typedef enum UDateTimeScale {
    /**
     * Used in the JDK. Data is a Java <code>long</code> (<code>int64_t</code>). Value
     * is milliseconds since January 1, 1970.
     *
     * @stable ICU 3.2
     */
    UDTS_JAVA_TIME = 0,

    /**
     * Used on Unix systems. Data is <code>int32_t</code> or <code>int64_t</code>. Value
     * is seconds since January 1, 1970.
     *
     * @stable ICU 3.2
     */
    UDTS_UNIX_TIME,
    
    /**
     * Used in IUC4C. Data is a <code>double</code>. Value
     * is milliseconds since January 1, 1970.
     *
     * @stable ICU 3.2
     */
    UDTS_ICU4C_TIME,
    
    /**
     * Used in Windows for file times. Data is an <code>int64_t</code>. Value
     * is ticks (1 tick == 100 nanoseconds) since January 1, 1601.
     *
     * @stable ICU 3.2
     */
    UDTS_WINDOWS_FILE_TIME,
    
    /**
     * Used in the .NET framework's <code>System.DateTime</code> structure. Data is an <code>int64_t</code>. Value
     * is ticks (1 tick == 100 nanoseconds) since January 1, 0001.
     *
     * @stable ICU 3.2
     */
    UDTS_DOTNET_DATE_TIME,
    
    /**
     * Used in older Macintosh systems. Data is <code>int32_t</code> or <code>int64_t</code>. Value
     * is seconds since January 1, 1904.
     *
     * @stable ICU 3.2
     */
    UDTS_MAC_OLD_TIME,
    
    /**
     * Used in newer Macintosh systems. Data is a <code>double</code>. Value
     * is seconds since January 1, 2001.
     *
     * @stable ICU 3.2
     */
    UDTS_MAC_TIME,
    
    /**
     * Used in Excel. Data is an <code>?unknown?</code>. Value
     * is days since December 31, 1899.
     *
     * @stable ICU 3.2
     */
    UDTS_EXCEL_TIME,
    
    /**
     * Used in DB2. Data is an <code>?unknown?</code>. Value
     * is days since December 31, 1899.
     *
     * @stable ICU 3.2
     */
    UDTS_DB2_TIME,

    /**
     * Data is a <code>long</code>. Value is microseconds since January 1, 1970.
     * Similar to Unix time (linear value from 1970) and struct timeval
     * (microseconds resolution).
     *
     * @stable ICU 3.8
     */
    UDTS_UNIX_MICROSECONDS_TIME,

    /**
     * The first unused time scale value. The limit of this enum
     */
    UDTS_MAX_SCALE
} UDateTimeScale;

typedef enum UTimeScaleValue {
    /**
     * The constant used to select the units vale
     * for a time scale.
     * 
     * @see utmscale_getTimeScaleValue
     *
     * @stable ICU 3.2
     */
    UTSV_UNITS_VALUE = 0,

    /**
     * The constant used to select the epoch offset value
     * for a time scale.
     * 
     * @see utmscale_getTimeScaleValue
     *
     * @stable ICU 3.2
     */
    UTSV_EPOCH_OFFSET_VALUE=1,

    /**
     * The constant used to select the minimum from value
     * for a time scale.
     * 
     * @see utmscale_getTimeScaleValue
     *
     * @stable ICU 3.2
     */
    UTSV_FROM_MIN_VALUE=2,

    /**
     * The constant used to select the maximum from value
     * for a time scale.
     * 
     * @see utmscale_getTimeScaleValue
     *
     * @stable ICU 3.2
     */
    UTSV_FROM_MAX_VALUE=3,

    /**
     * The constant used to select the minimum to value
     * for a time scale.
     * 
     * @see utmscale_getTimeScaleValue
     *
     * @stable ICU 3.2
     */
    UTSV_TO_MIN_VALUE=4,

    /**
     * The constant used to select the maximum to value
     * for a time scale.
     * 
     * @see utmscale_getTimeScaleValue
     *
     * @stable ICU 3.2
     */
    UTSV_TO_MAX_VALUE=5,

#ifndef U_HIDE_INTERNAL_API
    /**
     * The constant used to select the epoch plus one value
     * for a time scale.
     * 
     * NOTE: This is an internal value. DO NOT USE IT. May not
     * actually be equal to the epoch offset value plus one.
     * 
     * @see utmscale_getTimeScaleValue
     *
     * @internal ICU 3.2
     */
    UTSV_EPOCH_OFFSET_PLUS_1_VALUE=6,

    /**
     * The constant used to select the epoch plus one value
     * for a time scale.
     * 
     * NOTE: This is an internal value. DO NOT USE IT. May not
     * actually be equal to the epoch offset value plus one.
     * 
     * @see utmscale_getTimeScaleValue
     *
     * @internal ICU 3.2
     */
    UTSV_EPOCH_OFFSET_MINUS_1_VALUE=7,

    /**
     * The constant used to select the units round value
     * for a time scale.
     * 
     * NOTE: This is an internal value. DO NOT USE IT.
     * 
     * @see utmscale_getTimeScaleValue
     *
     * @internal ICU 3.2
     */
    UTSV_UNITS_ROUND_VALUE=8,

    /**
     * The constant used to select the minimum safe rounding value
     * for a time scale.
     * 
     * NOTE: This is an internal value. DO NOT USE IT.
     * 
     * @see utmscale_getTimeScaleValue
     *
     * @internal ICU 3.2
     */
    UTSV_MIN_ROUND_VALUE=9,

    /**
     * The constant used to select the maximum safe rounding value
     * for a time scale.
     * 
     * NOTE: This is an internal value. DO NOT USE IT.
     * 
     * @see utmscale_getTimeScaleValue
     *
     * @internal ICU 3.2
     */
    UTSV_MAX_ROUND_VALUE=10,

#endif /* U_HIDE_INTERNAL_API */

    /**
     * The number of time scale values, in other words limit of this enum.
     * 
     * @see utmscale_getTimeScaleValue
     */
    UTSV_MAX_SCALE_VALUE=11

} UTimeScaleValue;

U_STABLE int64_t U_EXPORT2
    utmscale_getTimeScaleValue(UDateTimeScale timeScale, UTimeScaleValue value, UErrorCode *status);

/* Conversion to 'universal time scale' */

U_STABLE int64_t U_EXPORT2
    utmscale_fromInt64(int64_t otherTime, UDateTimeScale timeScale, UErrorCode *status);

/* Conversion from 'universal time scale' */

U_STABLE int64_t U_EXPORT2
    utmscale_toInt64(int64_t universalTime, UDateTimeScale timeScale, UErrorCode *status);

#endif /* #if !UCONFIG_NO_FORMATTING */

#endif

