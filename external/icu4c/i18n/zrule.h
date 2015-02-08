#ifndef __ZRULE_H
#define __ZRULE_H


#include "unicode/utypes.h"

#ifndef UCNV_H

struct ZRule;
typedef struct ZRule ZRule;

struct IZRule;
typedef struct IZRule IZRule;

struct AZRule;
typedef struct AZRule AZRule;

#endif


U_DRAFT void U_EXPORT2
zrule_close(ZRule* rule);

U_DRAFT UBool U_EXPORT2
zrule_equals(const ZRule* rule1, const ZRule* rule2);

U_DRAFT void U_EXPORT2
zrule_getName(ZRule* rule, UChar* name, int32_t nameLength);

U_DRAFT int32_t U_EXPORT2
zrule_getRawOffset(ZRule* rule);

U_DRAFT int32_t U_EXPORT2
zrule_getDSTSavings(ZRule* rule);

U_DRAFT UBool U_EXPORT2
zrule_isEquivalentTo(ZRule* rule1,  ZRule* rule2);


U_DRAFT IZRule* U_EXPORT2
izrule_open(const UChar* name, int32_t nameLength, int32_t rawOffset, int32_t dstSavings);

U_DRAFT void U_EXPORT2
izrule_close(IZRule* rule);

U_DRAFT IZRule* U_EXPORT2
izrule_clone(IZRule *rule);

U_DRAFT UBool U_EXPORT2
izrule_equals(const IZRule* rule1, const IZRule* rule2);

U_DRAFT void U_EXPORT2
izrule_getName(IZRule* rule, UChar* & name, int32_t & nameLength);

U_DRAFT int32_t U_EXPORT2
izrule_getRawOffset(IZRule* rule);

U_DRAFT int32_t U_EXPORT2
izrule_getDSTSavings(IZRule* rule);

U_DRAFT UBool U_EXPORT2
izrule_isEquivalentTo(IZRule* rule1,  IZRule* rule2);

U_DRAFT UBool U_EXPORT2
izrule_getFirstStart(IZRule* rule, int32_t prevRawOffset, int32_t prevDSTSavings, 
                    UDate& result);

U_DRAFT UBool U_EXPORT2
izrule_getFinalStart(IZRule* rule, int32_t prevRawOffset, int32_t prevDSTSavings, 
                    UDate& result);

U_DRAFT UBool U_EXPORT2
izrule_getNextStart(IZRule* rule, UDate base, int32_t prevRawOffset, 
                   int32_t prevDSTSavings, UBool inclusive, UDate& result);

U_DRAFT UBool U_EXPORT2
izrule_getPreviousStart(IZRule* rule, UDate base, int32_t prevRawOffset, 
                       int32_t prevDSTSavings, UBool inclusive, UDate& result);


U_DRAFT UClassID U_EXPORT2
izrule_getStaticClassID(IZRule* rule);

U_DRAFT UClassID U_EXPORT2
izrule_getDynamicClassID(IZRule* rule);

#endif
