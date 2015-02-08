

#ifndef SVGFEGaussianBlurElement_h
#define SVGFEGaussianBlurElement_h

#if ENABLE(SVG) && ENABLE(FILTERS)
#include "FEGaussianBlur.h"
#include "SVGFilterPrimitiveStandardAttributes.h"

namespace WebCore {

    extern char SVGStdDeviationXAttrIdentifier[];
    extern char SVGStdDeviationYAttrIdentifier[];

    class SVGFEGaussianBlurElement : public SVGFilterPrimitiveStandardAttributes {
    public:
        SVGFEGaussianBlurElement(const QualifiedName&, Document*);
        virtual ~SVGFEGaussianBlurElement();

        void setStdDeviation(float stdDeviationX, float stdDeviationY);

        virtual void parseMappedAttribute(MappedAttribute*);
        virtual void synchronizeProperty(const QualifiedName&);
        virtual bool build(SVGResourceFilter*);

    private:
        DECLARE_ANIMATED_PROPERTY(SVGFEGaussianBlurElement, SVGNames::inAttr, String, In1, in1)
        DECLARE_ANIMATED_PROPERTY_MULTIPLE_WRAPPERS(SVGFEGaussianBlurElement, SVGNames::stdDeviationAttr, SVGStdDeviationXAttrIdentifier, float, StdDeviationX, stdDeviationX)
        DECLARE_ANIMATED_PROPERTY_MULTIPLE_WRAPPERS(SVGFEGaussianBlurElement, SVGNames::stdDeviationAttr, SVGStdDeviationYAttrIdentifier, float, StdDeviationY, stdDeviationY)
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif
