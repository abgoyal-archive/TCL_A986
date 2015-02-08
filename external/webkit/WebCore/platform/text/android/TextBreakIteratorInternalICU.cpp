

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
    // FIXME: Should use system locale.
    return "en_us";
}

}
