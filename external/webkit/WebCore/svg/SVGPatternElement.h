

#ifndef SVGPatternElement_h
#define SVGPatternElement_h

#if ENABLE(SVG)
#include "RenderObject.h"
#include "SVGExternalResourcesRequired.h"
#include "SVGFitToViewBox.h"
#include "SVGLangSpace.h"
#include "SVGPaintServerPattern.h"
#include "SVGStyledElement.h"
#include "SVGTests.h"
#include "SVGTransformList.h"
#include "SVGURIReference.h"

namespace WebCore {

    struct PatternAttributes;
 
    class SVGLength;

    class SVGPatternElement : public SVGStyledElement,
                              public SVGURIReference,
                              public SVGTests,
                              public SVGLangSpace,
                              public SVGExternalResourcesRequired,
                              public SVGFitToViewBox {
    public:
        SVGPatternElement(const QualifiedName&, Document*);
        virtual ~SVGPatternElement();
        
        virtual bool isValid() const { return SVGTests::isValid(); }

        virtual void parseMappedAttribute(MappedAttribute*);
        virtual void svgAttributeChanged(const QualifiedName&);
        virtual void synchronizeProperty(const QualifiedName&);
        virtual void childrenChanged(bool changedByParser = false, Node* beforeChange = 0, Node* afterChange = 0, int childCountDelta = 0);

        virtual RenderObject* createRenderer(RenderArena*, RenderStyle*);
        virtual SVGResource* canvasResource(const RenderObject*);

    private:
        DECLARE_ANIMATED_PROPERTY(SVGPatternElement, SVGNames::xAttr, SVGLength, X, x)
        DECLARE_ANIMATED_PROPERTY(SVGPatternElement, SVGNames::yAttr, SVGLength, Y, y)
        DECLARE_ANIMATED_PROPERTY(SVGPatternElement, SVGNames::widthAttr, SVGLength, Width, width)
        DECLARE_ANIMATED_PROPERTY(SVGPatternElement, SVGNames::heightAttr, SVGLength, Height, height)
        DECLARE_ANIMATED_PROPERTY(SVGPatternElement, SVGNames::patternUnitsAttr, int, PatternUnits, patternUnits)
        DECLARE_ANIMATED_PROPERTY(SVGPatternElement, SVGNames::patternContentUnitsAttr, int, PatternContentUnits, patternContentUnits)
        DECLARE_ANIMATED_PROPERTY(SVGPatternElement, SVGNames::patternTransformAttr, SVGTransformList*, PatternTransform, patternTransform)

        // SVGURIReference
        DECLARE_ANIMATED_PROPERTY(SVGPatternElement, XLinkNames::hrefAttr, String, Href, href)

        // SVGExternalResourcesRequired
        DECLARE_ANIMATED_PROPERTY(SVGPatternElement, SVGNames::externalResourcesRequiredAttr, bool, ExternalResourcesRequired, externalResourcesRequired)

        // SVGPatternElement
        DECLARE_ANIMATED_PROPERTY(SVGPatternElement, SVGNames::viewBoxAttr, FloatRect, ViewBox, viewBox)
        DECLARE_ANIMATED_PROPERTY(SVGPatternElement, SVGNames::preserveAspectRatioAttr, SVGPreserveAspectRatio, PreserveAspectRatio, preserveAspectRatio)
 
        mutable RefPtr<SVGPaintServerPattern> m_resource;

    private:
        friend class SVGPaintServerPattern;
        void buildPattern(const FloatRect& targetRect) const;

        PatternAttributes collectPatternProperties() const;
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif
