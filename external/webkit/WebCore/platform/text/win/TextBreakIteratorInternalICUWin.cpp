

#include "config.h"
#include "TextBreakIteratorInternalICU.h"

namespace WebCore {

const char* currentSearchLocaleID()
{
    // FIXME: Should use system locale.
    return "";
}

const char* currentTextBreakLocaleID()
{
    // Using en_US_POSIX now so word selection in address field works as expected as before (double-clicking
    // in a URL selects a word delimited by periods rather than selecting the entire URL).
    // However, this is not entirely correct - we should honor the system locale in the normal case.
    // FIXME: <rdar://problem/6786703> Should use system locale for text breaking
    return "en_US_POSIX";
}

}
