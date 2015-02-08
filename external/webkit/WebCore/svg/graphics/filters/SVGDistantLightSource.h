

#ifndef SVGDistantLightSource_h
#define SVGDistantLightSource_h

#if ENABLE(SVG) && ENABLE(FILTERS)
#include "SVGLightSource.h"

namespace WebCore {

    class DistantLightSource : public LightSource {
    public:
        static PassRefPtr<DistantLightSource> create(float azimuth, float elevation)
        {
            return adoptRef(new DistantLightSource(azimuth, elevation));
        }

        float azimuth() const { return m_azimuth; }
        float elevation() const { return m_elevation; }

        virtual TextStream& externalRepresentation(TextStream&) const;

    private:
        DistantLightSource(float azimuth, float elevation)
            : LightSource(LS_DISTANT)
            , m_azimuth(azimuth)
            , m_elevation(elevation)
        {
        }

        float m_azimuth;
        float m_elevation;
    };

} // namespace WebCore

#endif // ENABLE(SVG) && ENABLE(FILTERS)

#endif // SVGDistantLightSource_h
