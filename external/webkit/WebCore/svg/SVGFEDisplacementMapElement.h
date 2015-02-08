

#ifndef SVGFEDisplacementMapElement_h
#define SVGFEDisplacementMapElement_h

#if ENABLE(SVG) && ENABLE(FILTERS)
#include "SVGFEDisplacementMap.h"
#include "SVGFilterPrimitiveStandardAttributes.h"

namespace WebCore {
    
    class SVGFEDisplacementMapElement : public SVGFilterPrimitiveStandardAttributes {
    public:
        SVGFEDisplacementMapElement(const QualifiedName& tagName, Document*);
        virtual ~SVGFEDisplacementMapElement();
        
        static ChannelSelectorType stringToChannel(const String&);
        
        virtual void parseMappedAttribute(MappedAttribute*);
        virtual void synchronizeProperty(const QualifiedName&);
        virtual bool build(SVGResourceFilter*);
        
    private:
        DECLARE_ANIMATED_PROPERTY(SVGFEDisplacementMapElement, SVGNames::inAttr, String, In1, in1)
        DECLARE_ANIMATED_PROPERTY(SVGFEDisplacementMapElement, SVGNames::in2Attr, String, In2, in2)
        DECLARE_ANIMATED_PROPERTY(SVGFEDisplacementMapElement, SVGNames::xChannelSelectorAttr, int, XChannelSelector, xChannelSelector)
        DECLARE_ANIMATED_PROPERTY(SVGFEDisplacementMapElement, SVGNames::yChannelSelectorAttr, int, YChannelSelector, yChannelSelector)
        DECLARE_ANIMATED_PROPERTY(SVGFEDisplacementMapElement, SVGNames::scaleAttr, float, Scale, scale)
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif // SVGFEDisplacementMapElement_h
