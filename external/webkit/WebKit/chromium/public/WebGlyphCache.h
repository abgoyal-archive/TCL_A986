

#ifndef WebGlyphCache_h
#define WebGlyphCache_h

#include "WebCommon.h"

namespace WebKit {

// An interface to query WebKit's glyph cache (which maps unicode characters to
// glyphs in particular fonts).
class WebGlyphCache {
public:
    // Returns the number of pages in the glyph cache.
    WEBKIT_API static size_t pageCount();

private:
    WebGlyphCache();  // Not intended to be instanced.
};

}  // namespace WebKit

#endif
