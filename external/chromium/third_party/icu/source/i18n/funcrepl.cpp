
#include "unicode/utypes.h"

#if !UCONFIG_NO_TRANSLITERATION

#include "unicode/translit.h"
#include "unicode/uniset.h"
#include "funcrepl.h"

static const UChar AMPERSAND = 38; // '&'
static const UChar OPEN[]    = {40,32,0}; // "( "
static const UChar CLOSE[]   = {32,41,0}; // " )"

U_NAMESPACE_BEGIN

UOBJECT_DEFINE_RTTI_IMPLEMENTATION(FunctionReplacer)

FunctionReplacer::FunctionReplacer(Transliterator* adoptedTranslit,
                                   UnicodeFunctor* adoptedReplacer) {
    translit = adoptedTranslit;
    replacer = adoptedReplacer;
}

FunctionReplacer::FunctionReplacer(const FunctionReplacer& other) :
    UnicodeFunctor(other),
    UnicodeReplacer(other)
{
    translit = other.translit->clone();
    replacer = other.replacer->clone();
}

FunctionReplacer::~FunctionReplacer() {
    delete translit;
    delete replacer;
}

UnicodeFunctor* FunctionReplacer::clone() const {
    return new FunctionReplacer(*this);
}

UnicodeReplacer* FunctionReplacer::toReplacer() const {
    return (UnicodeReplacer*) this;
}

int32_t FunctionReplacer::replace(Replaceable& text,
                                  int32_t start,
                                  int32_t limit,
                                  int32_t& cursor)
{

    // First delegate to subordinate replacer
    int32_t len = replacer->toReplacer()->replace(text, start, limit, cursor);
    limit = start + len;

    // Now transliterate
    limit = translit->transliterate(text, start, limit);

    return limit - start;
}

UnicodeString& FunctionReplacer::toReplacerPattern(UnicodeString& rule,
                                                   UBool escapeUnprintable) const {
    UnicodeString str;
    rule.truncate(0);
    rule.append(AMPERSAND);
    rule.append(translit->getID());
    rule.append(OPEN);
    rule.append(replacer->toReplacer()->toReplacerPattern(str, escapeUnprintable));
    rule.append(CLOSE);
    return rule;
}

void FunctionReplacer::addReplacementSetTo(UnicodeSet& toUnionTo) const {
    UnicodeSet set;
    toUnionTo.addAll(translit->getTargetSet(set));
}

void FunctionReplacer::setData(const TransliterationRuleData* d) {
    replacer->setData(d);
}

U_NAMESPACE_END

#endif /* #if !UCONFIG_NO_TRANSLITERATION */

//eof
