

#include <CoreGraphics/CGContext.h>

namespace WebCore {

// FIXME: This would be in GraphicsContextCG.h if that existed.
CGColorSpaceRef deviceRGBColorSpaceRef();

// FIXME: This would be in GraphicsContextCG.h if that existed.
CGColorSpaceRef sRGBColorSpaceRef();

class GraphicsContextPlatformPrivate {
public:
    GraphicsContextPlatformPrivate(CGContextRef cgContext)
        : m_cgContext(cgContext)
#if PLATFORM(WIN)
        , m_hdc(0)
        , m_transparencyCount(0)
        , m_shouldIncludeChildWindows(false)
#endif
        , m_userToDeviceTransformKnownToBeIdentity(false)
    {
    }
    
    ~GraphicsContextPlatformPrivate()
    {
    }

#if PLATFORM(MAC) || PLATFORM(CHROMIUM)
    // These methods do nothing on Mac.
    void save() {}
    void restore() {}
    void flush() {}
    void clip(const FloatRect&) {}
    void clip(const Path&) {}
    void scale(const FloatSize&) {}
    void rotate(float) {}
    void translate(float, float) {}
    void concatCTM(const AffineTransform&) {}
    void beginTransparencyLayer() {}
    void endTransparencyLayer() {}
#endif

#if PLATFORM(WIN)
    // On Windows, we need to update the HDC for form controls to draw in the right place.
    void save();
    void restore();
    void flush();
    void clip(const FloatRect&);
    void clip(const Path&);
    void scale(const FloatSize&);
    void rotate(float);
    void translate(float, float);
    void concatCTM(const AffineTransform&);
    void beginTransparencyLayer() { m_transparencyCount++; }
    void endTransparencyLayer() { m_transparencyCount--; }

    HDC m_hdc;
    unsigned m_transparencyCount;
    bool m_shouldIncludeChildWindows;
#endif

    RetainPtr<CGContextRef> m_cgContext;
    bool m_userToDeviceTransformKnownToBeIdentity;
};

}
