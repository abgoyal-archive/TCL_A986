
#ifndef USCANF_H
#define USCANF_H

#include "unicode/utypes.h"

#if !UCONFIG_NO_FORMATTING

#include "unicode/ustdio.h"

U_CFUNC int32_t
u_scanf_parse(UFILE     *f,
            const UChar *patternSpecification,
            va_list     ap);

#endif /* #if !UCONFIG_NO_FORMATTING */

#endif

