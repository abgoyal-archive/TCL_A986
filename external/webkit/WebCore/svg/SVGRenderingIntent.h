

#ifndef SVGRenderingIntent_h
#define SVGRenderingIntent_h

#if ENABLE(SVG)

#include <wtf/RefCounted.h>

namespace WebCore {

class SVGRenderingIntent : public RefCounted<SVGRenderingIntent> {
public:
    enum SVGRenderingIntentType {
        RENDERING_INTENT_UNKNOWN                  = 0,
        RENDERING_INTENT_AUTO                     = 1,
        RENDERING_INTENT_PERCEPTUAL               = 2,
        RENDERING_INTENT_RELATIVE_COLORIMETRIC    = 3,
        RENDERING_INTENT_SATURATION               = 4,
        RENDERING_INTENT_ABSOLUTE_COLORIMETRIC    = 5
    };

private:
    SVGRenderingIntent() { }
};

} // namespace WebCore

#endif // ENABLE(SVG)
#endif // SVGRenderingIntent_h

// vim:ts=4:noet
