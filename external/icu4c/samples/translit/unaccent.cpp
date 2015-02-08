
#include "unaccent.h"

const char UnaccentTransliterator::fgClassID = 0;

UnaccentTransliterator::UnaccentTransliterator() :
    normalizer("", UNORM_NFD),
    Transliterator("Unaccent", 0) {
}

UnaccentTransliterator::~UnaccentTransliterator() {
}

UChar UnaccentTransliterator::unaccent(UChar c) const {
    UnicodeString str(c);
    UErrorCode status = U_ZERO_ERROR;
    UnaccentTransliterator* t = (UnaccentTransliterator*)this;

    t->normalizer.setText(str, status);
    if (U_FAILURE(status)) {
        return c;
    }
    return (UChar) t->normalizer.next();
}

void UnaccentTransliterator::handleTransliterate(Replaceable& text,
                                                 UTransPosition& index,
                                                 UBool incremental) const {
    UnicodeString str("a");
    while (index.start < index.limit) {
        UChar c = text.charAt(index.start);
        UChar d = unaccent(c);
        if (c != d) {
            str.setCharAt(0, d);
            text.handleReplaceBetween(index.start, index.start+1, str);
        }
        index.start++;
    }
}
