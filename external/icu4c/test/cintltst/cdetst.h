
#ifndef _CGERCOLLTST
#define _CGERCOLLTST

#include "unicode/utypes.h"

#if !UCONFIG_NO_COLLATION

#include "cintltst.h"


#define MAX_TOKEN_LEN 16

    /* performs test with strength PRIMARY */
static    void TestPrimary(void);

    /* perform test with strength TERTIARY */
static     void TestTertiary(void);


#endif /* #if !UCONFIG_NO_COLLATION */

#endif
