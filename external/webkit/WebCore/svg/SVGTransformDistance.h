

#ifndef SVGTransformDistance_h
#define SVGTransformDistance_h
#if ENABLE(SVG)

#include "SVGTransform.h"

namespace WebCore {
    
class AffineTransform;
    
class SVGTransformDistance {
public:
    SVGTransformDistance();
    SVGTransformDistance(const SVGTransform& fromTransform, const SVGTransform& toTransform);
    
    SVGTransformDistance scaledDistance(float scaleFactor) const;
    SVGTransform addToSVGTransform(const SVGTransform&) const;
    void addSVGTransform(const SVGTransform&, bool absoluteValue = false);
    
    static SVGTransform addSVGTransforms(const SVGTransform&, const SVGTransform&);
    
    bool isZero() const;
    
    float distance() const;
private:
    SVGTransformDistance(SVGTransform::SVGTransformType, float angle, float cx, float cy, const AffineTransform&);
        
    SVGTransform::SVGTransformType m_type;
    float m_angle;
    float m_cx;
    float m_cy;
    AffineTransform m_transform; // for storing scale, translation or matrix transforms
};
}

#endif // ENABLE(SVG)
#endif // SVGTransformDistance_h
