

#ifndef SkiaWinOutlineCache_h
#define SkiaWinOutlineCache_h

#include <windows.h>
#include <usp10.h>

class GraphicsContext;
class SkPath;
class SkPoint;
class PlatformContextSkia;

namespace WebCore {

// FIXME: Rename file to SkiaWinOutlineCache
class SkiaWinOutlineCache {
public:
    static const SkPath* lookupOrCreatePathForGlyph(HDC, HFONT, WORD);
    // Removes any cached glyphs from the outline cache corresponding to the
    // given font handle.
    static void removePathsForFont(HFONT);

private:
    SkiaWinOutlineCache();
};

// The functions below are used for more complex font drawing (effects such as
// stroking and more complex transforms) than Windows supports directly.  Since 
// Windows drawing is faster you should use windowsCanHandleTextDrawing first to 
// check if using Skia is required at all.
// Note that the text will look different (no ClearType) so this should only be
// used when necessary.
//
// When you call a Skia* text drawing function, various glyph outlines will be
// cached. As a result, you should call SkiaWinOutlineCache::removePathsForFont
// when the font is destroyed so that the cache does not outlive the font (since
// the HFONTs are recycled).
//
// Remember that Skia's text drawing origin is the baseline, like WebKit, not
// the top, like Windows.

// Returns true if the fillColor and shadowColor are opaque and the text-shadow
// is not blurred.
bool windowsCanHandleDrawTextShadow(GraphicsContext*);

// Returns true if advanced font rendering is recommended.
bool windowsCanHandleTextDrawing(GraphicsContext*);

// Note that the offsets parameter is optional.  If not NULL it represents a
// per glyph offset (such as returned by ScriptPlace Windows API function).
//
// Returns true of the text was drawn successfully. False indicates an error
// from Windows.
bool paintSkiaText(GraphicsContext* graphicsContext,
                   HFONT hfont,
                   int numGlyphs,
                   const WORD* glyphs,
                   const int* advances,
                   const GOFFSET* offsets,
                   const SkPoint* origin);

}  // namespace WebCore

#endif  // SkiaWinOutlineCache_h
