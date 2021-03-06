
#include "unicode/utypes.h"

#if !UCONFIG_NO_TRANSLITERATION

#include "remtrans.h"
#include "unicode/unifilt.h"

static const UChar CURR_ID[] = {65, 110, 121, 45, 0x52, 0x65, 0x6D, 0x6F, 0x76, 0x65, 0x00}; /* "Any-Remove" */

U_NAMESPACE_BEGIN

UOBJECT_DEFINE_RTTI_IMPLEMENTATION(RemoveTransliterator)

static Transliterator* RemoveTransliterator_create(const UnicodeString& /*ID*/,
                                                   Transliterator::Token /*context*/) {
    /* We don't need the ID or context. We just remove data */
    return new RemoveTransliterator();
}

void RemoveTransliterator::registerIDs() {

    Transliterator::_registerFactory(::CURR_ID, RemoveTransliterator_create, integerToken(0));

    Transliterator::_registerSpecialInverse(UNICODE_STRING_SIMPLE("Remove"),
                                            UNICODE_STRING_SIMPLE("Null"), FALSE);
}

RemoveTransliterator::RemoveTransliterator() : Transliterator(::CURR_ID, 0) {}

RemoveTransliterator::~RemoveTransliterator() {}

Transliterator* RemoveTransliterator::clone(void) const {
    Transliterator* result = new RemoveTransliterator();
    if (result != NULL && getFilter() != 0) {
        result->adoptFilter((UnicodeFilter*)(getFilter()->clone()));
    }
    return result;
}

void RemoveTransliterator::handleTransliterate(Replaceable& text, UTransPosition& index,
                                               UBool /*isIncremental*/) const {
    // Our caller (filteredTransliterate) has already narrowed us
    // to an unfiltered run.  Delete it.
    UnicodeString empty;
    text.handleReplaceBetween(index.start, index.limit, empty);
    int32_t len = index.limit - index.start;
    index.contextLimit -= len;
    index.limit -= len;
}
U_NAMESPACE_END

#endif /* #if !UCONFIG_NO_TRANSLITERATION */
