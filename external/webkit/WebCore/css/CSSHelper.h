

#ifndef CSSHelper_h
#define CSSHelper_h

namespace WebCore {

    class String;

    // Used in many inappropriate contexts throughout WebCore. We'll have to examine and test
    // each call site to find out whether it needs the various things this function does. That
    // includes trimming leading and trailing control characters (including whitespace), removing
    // url() or URL() if it surrounds the entire string, removing matching quote marks if present,
    // and stripping all characters in the range U+0000-U+000C. Probably no caller needs this.
    String deprecatedParseURL(const String&);

    // We always assume 96 CSS pixels in a CSS inch. This is the cold hard truth of the Web.
    // At high DPI, we may scale a CSS pixel, but the ratio of the CSS pixel to the so-called
    // "absolute" CSS length units like inch and pt is always fixed and never changes.
    const float cssPixelsPerInch = 96;

} // namespace WebCore

#endif // CSSHelper_h
