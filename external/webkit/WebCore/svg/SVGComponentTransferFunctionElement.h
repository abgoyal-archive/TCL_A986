

#ifndef SVGComponentTransferFunctionElement_h
#define SVGComponentTransferFunctionElement_h

#if ENABLE(SVG) && ENABLE(FILTERS)
#include "SVGElement.h"
#include "SVGNames.h"
#include "SVGNumberList.h"
#include "FEComponentTransfer.h"

namespace WebCore {

    class SVGComponentTransferFunctionElement : public SVGElement {
    public:
        SVGComponentTransferFunctionElement(const QualifiedName&, Document*);
        virtual ~SVGComponentTransferFunctionElement();

        virtual void parseMappedAttribute(MappedAttribute*);
        virtual void synchronizeProperty(const QualifiedName&);
        
        ComponentTransferFunction transferFunction() const;

    private:
        DECLARE_ANIMATED_PROPERTY(SVGComponentTransferFunctionElement, SVGNames::typeAttr, int, Type, type)
        DECLARE_ANIMATED_PROPERTY(SVGComponentTransferFunctionElement, SVGNames::tableValuesAttr, SVGNumberList*, TableValues, tableValues)
        DECLARE_ANIMATED_PROPERTY(SVGComponentTransferFunctionElement, SVGNames::slopeAttr, float, Slope, slope)
        DECLARE_ANIMATED_PROPERTY(SVGComponentTransferFunctionElement, SVGNames::interceptAttr, float, Intercept, intercept)
        DECLARE_ANIMATED_PROPERTY(SVGComponentTransferFunctionElement, SVGNames::amplitudeAttr, float, Amplitude, amplitude)
        DECLARE_ANIMATED_PROPERTY(SVGComponentTransferFunctionElement, SVGNames::exponentAttr, float, Exponent, exponent)
        DECLARE_ANIMATED_PROPERTY(SVGComponentTransferFunctionElement, SVGNames::offsetAttr, float, Offset, offset)
    };

} // namespace WebCore

#endif // ENABLE(SVG) && ENABLE(FILTERS)
#endif
