
#include "unicode/utypes.h"

#if !UCONFIG_NO_TRANSLITERATION

#include "unicode/uchar.h"
#include "unicode/ustring.h"
#include "tolowtrn.h"
#include "ustr_imp.h"
#include "cpputils.h"

U_NAMESPACE_BEGIN

UOBJECT_DEFINE_RTTI_IMPLEMENTATION(LowercaseTransliterator)

LowercaseTransliterator::LowercaseTransliterator() : 
    CaseMapTransliterator(UNICODE_STRING("Any-Lower", 9), ucase_toFullLower)
{
}

LowercaseTransliterator::~LowercaseTransliterator() {
}

LowercaseTransliterator::LowercaseTransliterator(const LowercaseTransliterator& o) :
    CaseMapTransliterator(o)
{
}


Transliterator* LowercaseTransliterator::clone(void) const {
    return new LowercaseTransliterator(*this);
}

U_NAMESPACE_END

#endif /* #if !UCONFIG_NO_TRANSLITERATION */
