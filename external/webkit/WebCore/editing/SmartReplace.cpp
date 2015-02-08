

#include "config.h"
#include "SmartReplace.h"

#if !PLATFORM(CF) && !USE(ICU_UNICODE)

namespace WebCore {

bool isCharacterSmartReplaceExempt(UChar32 c, bool isPreviousCharacter)
{
    return false;
}

}

#endif // !PLATFORM(CF)
