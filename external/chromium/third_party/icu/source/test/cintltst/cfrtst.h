
#ifndef _CFRCOLLTST
#define _CFRCOLLTST

#include "unicode/utypes.h"

#if !UCONFIG_NO_COLLATION

#include "cintltst.h"

#define MAX_TOKEN_LEN 16

    /* performs Extra tests*/
   static void TestExtra(void);

    /* perform test with strength SECONDARY*/
 static   void TestSecondary(void);

    /*perform test with strength TERTIARY*/
static    void TestTertiary(void);


#endif /* #if !UCONFIG_NO_COLLATION */

#endif
