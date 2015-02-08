/* C API TEST FOR NUMBER FORMAT */
#ifndef _CNUMFRMTST
#define _CNUMFRMTST

#include "unicode/utypes.h"

#if !UCONFIG_NO_FORMATTING

#include "cintltst.h"


static void TestNumberFormat(void);

static void TestSpelloutNumberParse(void);

static void TestSignificantDigits(void);

static void TestNumberFormatPadding(void);

static void TestInt64Format(void);

static void TestNonExistentCurrency(void);

static void TestRBNFFormat(void);

static void TestCurrencyRegression(void);


#endif /* #if !UCONFIG_NO_FORMATTING */

#endif
