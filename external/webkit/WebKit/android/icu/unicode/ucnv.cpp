

// BEGIN android-added
// Add config.h to avoid compiler error in uobject.h
// ucnv.h includes uobject.h indirectly and uobjetcs.h defines new/delete.
// new/delete are also defined in WebCorePrefix.h which auto included in Android make.
//
// config.h has to be on top of the include list.
#include "config.h"
// END android-added

#include "EmojiFont.h"
#include <icu4c/common/unicode/ucnv.h>

namespace android {

U_STABLE UConverter* U_EXPORT2
ucnv_open_emoji(const char *converterName, UErrorCode *err) {
    if (EmojiFont::IsAvailable()) {
        if (strcmp(converterName, "Shift_JIS") == 0) {
            converterName = EmojiFont::GetShiftJisConverterName();
        }
    }
    return ucnv_open(converterName, err);
}

} // end namespace android
