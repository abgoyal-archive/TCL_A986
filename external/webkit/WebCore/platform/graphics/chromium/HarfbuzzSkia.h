

#ifndef HarfbuzzSkia_h
#define HarfbuzzSkia_h

extern "C" {
#include "harfbuzz-shaper.h"
#include "harfbuzz-unicode.h"
}

namespace WebCore {
    HB_Error harfbuzzSkiaGetTable(void* voidface, const HB_Tag, HB_Byte* buffer, HB_UInt* len);
    extern const HB_FontClass harfbuzzSkiaClass;
}  // namespace WebCore

#endif
