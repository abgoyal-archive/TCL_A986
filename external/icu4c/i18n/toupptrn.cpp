
#include "unicode/utypes.h"

#if !UCONFIG_NO_TRANSLITERATION

#include "unicode/ustring.h"
#include "unicode/uchar.h"
#include "toupptrn.h"
#include "ustr_imp.h"
#include "cpputils.h"

U_NAMESPACE_BEGIN

UOBJECT_DEFINE_RTTI_IMPLEMENTATION(UppercaseTransliterator)

UppercaseTransliterator::UppercaseTransliterator() :
    CaseMapTransliterator(UNICODE_STRING("Any-Upper", 9), ucase_toFullUpper)
{
}

UppercaseTransliterator::~UppercaseTransliterator() {
}

UppercaseTransliterator::UppercaseTransliterator(const UppercaseTransliterator& o) :
    CaseMapTransliterator(o)
{
}


Transliterator* UppercaseTransliterator::clone(void) const {
    return new UppercaseTransliterator(*this);
}

U_NAMESPACE_END

#endif /* #if !UCONFIG_NO_TRANSLITERATION */
