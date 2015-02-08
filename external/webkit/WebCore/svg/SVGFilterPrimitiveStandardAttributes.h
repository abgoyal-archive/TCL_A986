

#ifndef SVGFilterPrimitiveStandardAttributes_h
#define SVGFilterPrimitiveStandardAttributes_h

#if ENABLE(SVG) && ENABLE(FILTERS)
#include "SVGFilterBuilder.h"
#include "SVGNames.h"
#include "SVGResourceFilter.h"
#include "SVGStyledElement.h"

namespace WebCore {

    class SVGResourceFilter;

    class SVGFilterPrimitiveStandardAttributes : public SVGStyledElement {
    public:
        SVGFilterPrimitiveStandardAttributes(const QualifiedName&, Document*);
        virtual ~SVGFilterPrimitiveStandardAttributes();
        
        virtual bool isFilterEffect() const { return true; }

        virtual void parseMappedAttribute(MappedAttribute*);
        virtual void synchronizeProperty(const QualifiedName&);
        virtual bool build(SVGResourceFilter*) = 0;

        virtual bool rendererIsNeeded(RenderStyle*) { return false; }

    protected:
        friend class SVGResourceFilter;
        void setStandardAttributes(SVGResourceFilter*, FilterEffect*) const;

    private:
        DECLARE_ANIMATED_PROPERTY(SVGFilterPrimitiveStandardAttributes, SVGNames::xAttr, SVGLength, X, x)
        DECLARE_ANIMATED_PROPERTY(SVGFilterPrimitiveStandardAttributes, SVGNames::yAttr, SVGLength, Y, y)
        DECLARE_ANIMATED_PROPERTY(SVGFilterPrimitiveStandardAttributes, SVGNames::widthAttr, SVGLength, Width, width)
        DECLARE_ANIMATED_PROPERTY(SVGFilterPrimitiveStandardAttributes, SVGNames::heightAttr, SVGLength, Height, height)
        DECLARE_ANIMATED_PROPERTY(SVGFilterPrimitiveStandardAttributes, SVGNames::resultAttr, String, Result, result)
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif
