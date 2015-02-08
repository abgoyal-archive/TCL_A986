

#include "config.h"
#include "WebFontRendering.h"

#include "FontPlatformData.h"

using WebCore::FontPlatformData;

namespace WebKit {

// static
void WebFontRendering::setHinting(SkPaint::Hinting hinting)
{
    FontPlatformData::setHinting(hinting);
}

// static
void WebFontRendering::setAntiAlias(bool isAntiAlias)
{
    FontPlatformData::setAntiAlias(isAntiAlias);
}

// static
void WebFontRendering::setSubpixelGlyphs(bool isSubpixelGlyphs)
{
    FontPlatformData::setSubpixelGlyphs(isSubpixelGlyphs);
}

// static
void WebFontRendering::setLCDOrder(SkFontHost::LCDOrder order)
{
    SkFontHost::SetSubpixelOrder(order);
}

// static
void WebFontRendering::setLCDOrientation(SkFontHost::LCDOrientation orientation)
{
    SkFontHost::SetSubpixelOrientation(orientation);
}

} // namespace WebKit
