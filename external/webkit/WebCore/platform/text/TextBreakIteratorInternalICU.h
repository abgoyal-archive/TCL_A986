

#ifndef TextBreakIteratorInternalICU_h
#define TextBreakIteratorInternalICU_h

// FIXME: Now that this handles locales for ICU, not just for text breaking,
// this file and the various implementation files should be renamed.

namespace WebCore {

    const char* currentSearchLocaleID();
    const char* currentTextBreakLocaleID();

}

#endif
