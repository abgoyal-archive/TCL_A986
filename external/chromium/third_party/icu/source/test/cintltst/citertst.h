

#ifndef _CITERCOLLTST
#define _CITERCOLLTST

#include "unicode/utypes.h"

#if !UCONFIG_NO_COLLATION

#include "unicode/ucol.h"
#include "unicode/ucoleitr.h"
#include "cintltst.h"

#define MAX_TOKEN_LEN 16

static void TestUnicodeChar(void);
static void TestNormalizedUnicodeChar(void);
static void TestNormalization(void);
 /**
 * Test for CollationElementIterator.previous()
 *
 * @bug 4108758 - Make sure it works with contracting characters
 * 
 */
static void TestPrevious(void);

static void TestOffset(void);
static void TestSetText(void);
static void TestMaxExpansion(void);
static void TestBug672(void);

static void TestBug672Normalize(void);
static void TestSmallBuffer(void);
static void TestCEs(void);
static void TestDiscontiguos(void);
static void TestCEBufferOverflow(void);
static void TestCEValidity(void);
static void TestSortKeyValidity(void);



static void assertEqual(UCollationElements *i1, UCollationElements *i2);


#endif /* #if !UCONFIG_NO_COLLATION */

#endif
