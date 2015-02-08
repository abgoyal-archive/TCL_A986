

#ifndef WebFontCache_h
#define WebFontCache_h

#include "WebCommon.h"

namespace WebKit {

// An interface to query and configure WebKit's font cache.
class WebFontCache {
public:
    // Returns the number of items in the font data cache.
    WEBKIT_API static size_t fontDataCount();

    // Returns the number of inactive items in the font data cache.
    WEBKIT_API static size_t inactiveFontDataCount();

    // Clears the cache.
    WEBKIT_API static void clear();

private:
    WebFontCache();  // Not intended to be instanced.
};

}  // namespace WebKit

#endif
