
/** C Utilities to aid in debugging **/

#ifndef _UDBGUTIL_H
#define _UDBGUTIL_H

#include "unicode/testtype.h"
#include "unicode/utypes.h"


enum UDebugEnumType {
    UDBG_UDebugEnumType = 0, /* Self-referential, strings for UDebugEnumType. Count=ENUM_COUNT. */
#if !UCONFIG_NO_FORMATTING
    UDBG_UCalendarDateFields, /* UCalendarDateFields. Count=UCAL_FIELD_COUNT.  Unsupported if UCONFIG_NO_FORMATTING. */
    UDBG_UCalendarMonths, /* UCalendarMonths. Count= (UCAL_UNDECIMBER+1) */
    UDBG_UDateFormatStyle, /* Count = UDAT_SHORT=1 */
#endif
    UDBG_UPlugReason,   /* Count = UPLUG_REASON_COUNT */
    UDBG_UPlugLevel,    /* COUNT = UPLUG_LEVEL_COUNT */
    UDBG_UAcceptResult, /* Count = ULOC_ACCEPT_FALLBACK+1=3 */
    
    /* All following enums may be discontiguous. */ 
    
#if !UCONFIG_NO_COLLATION
    UDBG_UColAttributeValue,  /* UCOL_ATTRIBUTE_VALUE_COUNT */
#endif
    UDBG_ENUM_COUNT,
    UDBG_HIGHEST_CONTIGUOUS_ENUM = UDBG_UAcceptResult,  /**< last enum in this list with contiguous (testable) values. */
    UDBG_INVALID_ENUM = -1 /** Invalid enum value **/
};

typedef enum UDebugEnumType UDebugEnumType;

T_CTEST_API int32_t T_CTEST_EXPORT2 udbg_enumCount(UDebugEnumType type);

T_CTEST_API const char * T_CTEST_EXPORT2 udbg_enumName(UDebugEnumType type, int32_t field);

T_CTEST_API int32_t T_CTEST_EXPORT2 udbg_enumExpectedCount(UDebugEnumType type);

T_CTEST_API int32_t T_CTEST_EXPORT2 udbg_enumArrayValue(UDebugEnumType type, int32_t field);

T_CTEST_API int32_t T_CTEST_EXPORT2 udbg_enumByName(UDebugEnumType type, const char *name);

#endif
