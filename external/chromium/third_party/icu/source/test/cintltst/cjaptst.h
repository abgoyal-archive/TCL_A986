
#ifndef _CJACOLLTST
#define _CJACOLLTST

#include "unicode/utypes.h"

#if !UCONFIG_NO_COLLATION

#include "cintltst.h"


#define MAX_TOKEN_LEN 16


     /*perform test with strength TERTIARY*/
static    void TestTertiary(void);

/* Testing base letters */
static void TestBase(void);

/* Testing plain, Daku-ten, Handaku-ten letters */
static void TestPlainDakutenHandakuten(void);

static void TestSmallLarge(void); 

static void TestKatakanaHiragana(void);

static void TestChooonKigoo(void);

#endif /* #if !UCONFIG_NO_COLLATION */

#endif
