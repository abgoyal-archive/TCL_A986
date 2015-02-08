
#ifndef U_ASSERT_H
#define U_ASSERT_H
/* utypes.h is included to get the proper define for uint8_t */
#include "unicode/utypes.h"
#if U_RELEASE
#define U_ASSERT(exp)
#else
#include <assert.h>
#define U_ASSERT(exp) assert(exp)
#endif
#endif


