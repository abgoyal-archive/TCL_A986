#ifndef _CLOCTEST
#define _CLOCTEST

#include "cintltst.h"
/*C API TEST FOR LOCALE */

static void TestBasicGetters(void);
static void TestPrefixes(void);
static void TestSimpleResourceInfo(void);
static  void TestDisplayNames(void);
 static  void TestGetAvailableLocales(void);
 static void TestDataDirectory(void);
 static void TestISOFunctions(void);
 static void TestISO3Fallback(void);
 static void TestUninstalledISO3Names(void);
 static void TestObsoleteNames(void);
 static void TestSimpleDisplayNames(void);
 static void TestVariantParsing(void);

 /* Test getting keyword enumeratin */
 static void TestKeywordVariants(void);

 static void TestKeywordSet(void);
 static void TestKeywordSetError(void);

 /* Test getting keyword values */
 static void TestKeywordVariantParsing(void);
 
 /* Test warning for no data in getDisplay* */
 static void TestDisplayNameWarning(void);

 /* Test uloc_getLocaleForLCID */
 static void TestGetLocaleForLCID(void);

 static void doTestDisplayNames(const char* inLocale, int32_t compareIndex);

 static void TestCanonicalization(void);

 static void TestDisplayKeywords(void);

 static void TestDisplayKeywordValues(void);

 static void TestGetBaseName(void);

static void TestGetLocale(void);

static void setUpDataTable(void);
static void cleanUpDataTable(void);
/*static void displayDataTable(void);*/
static void TestAcceptLanguage(void);

static void TestCalendar(void); 
static void TestDateFormat(void);
static void TestCollation(void);
static void TestULocale(void);
static void TestUResourceBundle(void);
static void TestDisplayName(void);

static void TestAcceptLanguage(void);

static void TestOrientation(void);

static void TestLikelySubtags(void);

static void TestForLanguageTag(void);
static void TestToLanguageTag(void);
#endif
