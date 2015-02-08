

#ifndef WebFontRendering_h
#define WebFontRendering_h

#include "../WebCommon.h"
#include <SkFontHost.h>
#include <SkPaint.h>

namespace WebKit {

class WebFontRendering {
public:
    // Set global font renderering preferences.

    WEBKIT_API static void setHinting(SkPaint::Hinting);
    WEBKIT_API static void setAntiAlias(bool on);
    WEBKIT_API static void setSubpixelGlyphs(bool on);
    WEBKIT_API static void setLCDOrder(SkFontHost::LCDOrder);
    WEBKIT_API static void setLCDOrientation(SkFontHost::LCDOrientation);
};

} // namespace WebKit

#endif
