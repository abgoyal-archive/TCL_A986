

#ifndef SVGLightSource_h
#define SVGLightSource_h

#if ENABLE(SVG) && ENABLE(FILTERS)
#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>

namespace WebCore {

    enum LightType {
        LS_DISTANT,
        LS_POINT,
        LS_SPOT
    };

    class TextStream;

    class LightSource : public RefCounted<LightSource> {
    public:
        LightSource(LightType type)
            : m_type(type)
        { }

        virtual ~LightSource() { }

        LightType type() const { return m_type; }
        virtual TextStream& externalRepresentation(TextStream&) const = 0;

    private:
        LightType m_type;
    };

} // namespace WebCore

#endif // ENABLE(SVG) && ENABLE(FILTERS)

#endif // SVGLightSource_h
