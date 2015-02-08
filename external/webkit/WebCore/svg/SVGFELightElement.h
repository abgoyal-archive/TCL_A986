

#ifndef SVGFELightElement_h
#define SVGFELightElement_h

#if ENABLE(SVG) && ENABLE(FILTERS)
#include "SVGElement.h"
#include "SVGLightSource.h"
#include "SVGNames.h"

namespace WebCore {

    class SVGFELightElement : public SVGElement {
    public:
        SVGFELightElement(const QualifiedName&, Document*);
        virtual ~SVGFELightElement();
        
        virtual PassRefPtr<LightSource> lightSource() const = 0;
        virtual void parseMappedAttribute(MappedAttribute*);
        virtual void synchronizeProperty(const QualifiedName&);

    private:
        DECLARE_ANIMATED_PROPERTY(SVGFELightElement, SVGNames::azimuthAttr, float, Azimuth, azimuth)
        DECLARE_ANIMATED_PROPERTY(SVGFELightElement, SVGNames::elevationAttr, float, Elevation, elevation)
        DECLARE_ANIMATED_PROPERTY(SVGFELightElement, SVGNames::xAttr, float, X, x)
        DECLARE_ANIMATED_PROPERTY(SVGFELightElement, SVGNames::yAttr, float, Y, y)
        DECLARE_ANIMATED_PROPERTY(SVGFELightElement, SVGNames::zAttr, float, Z, z)
        DECLARE_ANIMATED_PROPERTY(SVGFELightElement, SVGNames::pointsAtXAttr, float, PointsAtX, pointsAtX)
        DECLARE_ANIMATED_PROPERTY(SVGFELightElement, SVGNames::pointsAtYAttr, float, PointsAtY, pointsAtY)
        DECLARE_ANIMATED_PROPERTY(SVGFELightElement, SVGNames::pointsAtZAttr, float, PointsAtZ, pointsAtZ)
        DECLARE_ANIMATED_PROPERTY(SVGFELightElement, SVGNames::specularExponentAttr, float, SpecularExponent, specularExponent)
        DECLARE_ANIMATED_PROPERTY(SVGFELightElement, SVGNames::limitingConeAngleAttr, float, LimitingConeAngle, limitingConeAngle)
    };

} // namespace WebCore

#endif // ENABLE(SVG) && ENABLE(FILTERS)
#endif
