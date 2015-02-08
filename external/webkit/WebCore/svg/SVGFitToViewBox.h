

#ifndef SVGFitToViewBox_h
#define SVGFitToViewBox_h

#if ENABLE(SVG)
#include "SVGElement.h"
#include "SVGPreserveAspectRatio.h"

namespace WebCore {

class AffineTransform;

class SVGFitToViewBox {
public:
    SVGFitToViewBox();
    virtual ~SVGFitToViewBox();

    bool parseViewBox(Document*, const UChar*& start, const UChar* end, float& x, float& y, float& w, float& h, bool validate = true);
    static AffineTransform viewBoxToViewTransform(const FloatRect& viewBoxRect, const SVGPreserveAspectRatio&, float viewWidth, float viewHeight);

    bool parseMappedAttribute(Document*, MappedAttribute*);
    bool isKnownAttribute(const QualifiedName&);

    virtual void setViewBoxBaseValue(SVGAnimatedPropertyTraits<FloatRect>::PassType) = 0;
    virtual void setPreserveAspectRatioBaseValue(SVGAnimatedPropertyTraits<SVGPreserveAspectRatio>::PassType) = 0;
};

} // namespace WebCore

#endif // ENABLE(SVG)
#endif // SVGFitToViewBox_h
