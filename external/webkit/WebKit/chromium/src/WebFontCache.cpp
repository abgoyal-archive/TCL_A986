

#include "config.h"
#include "WebFontCache.h"

#include "FontCache.h"

using namespace WebCore;

namespace WebKit {

// static
size_t WebFontCache::fontDataCount()
{
    return fontCache()->fontDataCount();
}

// static
size_t WebFontCache::inactiveFontDataCount()
{
    return fontCache()->inactiveFontDataCount();
}

// static
void WebFontCache::clear()
{
    fontCache()->invalidate();
}

}  // namespace WebKit
