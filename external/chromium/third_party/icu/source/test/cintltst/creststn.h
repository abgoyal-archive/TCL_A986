#ifndef _CRESTSTN
#define _CRESTSTN
/* C TEST FOR NEW RESOURCEBUNDLE API*/
#include "cintltst.h"

extern const UChar *
tres_getString(const UResourceBundle *resB,
               int32_t index, const char *key,
               int32_t *length,
               UErrorCode *status);

void addNEWResourceBundleTest(TestNode**);

static void TestResourceBundles(void);
static void TestConstruction1(void);

static void TestAliasConflict(void);

static void TestFallback(void);

static void TestBinaryCollationData(void);

static void TestNewTypes(void);

static void TestEmptyTypes(void);

static void TestAPI(void);

static void TestErrorConditions(void);

static void TestGetVersion(void);

static void TestGetVersionColl(void);

static void TestEmptyBundle(void);

static void TestDirectAccess(void);

static void TestResourceLevelAliasing(void);

static void TestErrorCodes(void);

static void TestJB3763(void);

static void TestXPath(void);

static void TestStackReuse(void);

static UBool testTag(const char* frag, UBool in_Root, UBool in_te, UBool in_te_IN);

static void record_pass(void);
static void record_fail(void);


#endif
