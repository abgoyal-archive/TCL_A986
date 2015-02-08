

#include "GraphicsContext.h"

#include <cairo.h>
#include <math.h>
#include <stdio.h>
#include <wtf/MathExtras.h>

#if PLATFORM(GTK)
#include <pango/pango.h>
typedef struct _GdkExposeEvent GdkExposeEvent;
#elif PLATFORM(WIN)
#include <cairo-win32.h>
#endif

namespace WebCore {

class GraphicsContextPlatformPrivate {
public:
    GraphicsContextPlatformPrivate()
        : cr(0)
#if PLATFORM(GTK)
        , expose(0)
#elif PLATFORM(WIN)
        // NOTE:  These may note be needed: review and remove once Cairo implementation is complete
        , m_hdc(0)
        , m_transparencyCount(0)
        , m_shouldIncludeChildWindows(false)
#endif
    {
    }

    ~GraphicsContextPlatformPrivate()
    {
        cairo_destroy(cr);
    }

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
    void concatCTM(const TransformationMatrix&);
    void beginTransparencyLayer() { m_transparencyCount++; }
    void endTransparencyLayer() { m_transparencyCount--; }
    void syncContext(PlatformGraphicsContext* cr);
#else
    // On everything else, we do nothing.
    void save() {}
    void restore() {}
    void flush() {}
    void clip(const FloatRect&) {}
    void clip(const Path&) {}
    void scale(const FloatSize&) {}
    void rotate(float) {}
    void translate(float, float) {}
    void concatCTM(const AffineTransform&) {}
    void concatCTM(const TransformationMatrix&) {}
    void beginTransparencyLayer() {}
    void endTransparencyLayer() {}
    void syncContext(PlatformGraphicsContext* cr) {}
#endif

    cairo_t* cr;
    Vector<float> layers;

#if PLATFORM(GTK)
    GdkEventExpose* expose;
#elif PLATFORM(WIN)
    HDC m_hdc;
    unsigned m_transparencyCount;
    bool m_shouldIncludeChildWindows;
#endif
};

} // namespace WebCore

