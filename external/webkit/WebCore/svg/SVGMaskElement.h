

#ifndef SVGMaskElement_h
#define SVGMaskElement_h

#if ENABLE(SVG)
#include "RenderObject.h"
#include "SVGResourceMasker.h"
#include "SVGExternalResourcesRequired.h"
#include "SVGLangSpace.h"
#include "SVGStyledLocatableElement.h"
#include "SVGTests.h"
#include "SVGURIReference.h"
#include <wtf/HashMap.h>
#include <wtf/PassOwnPtr.h>

namespace WebCore {

    class SVGLength;
    class SVGResourceMasker;

    class SVGMaskElement : public SVGStyledLocatableElement,
                           public SVGURIReference,
                           public SVGTests,
                           public SVGLangSpace,
                           public SVGExternalResourcesRequired {
    public:
        SVGMaskElement(const QualifiedName&, Document*);
        virtual ~SVGMaskElement();
        virtual bool isValid() const { return SVGTests::isValid(); }

        FloatRect maskBoundingBox(const FloatRect&) const;
        virtual void parseMappedAttribute(MappedAttribute*);
        virtual void svgAttributeChanged(const QualifiedName&);
        virtual void synchronizeProperty(const QualifiedName&);
        virtual void childrenChanged(bool changedByParser = false, Node* beforeChange = 0, Node* afterChange = 0, int childCountDelta = 0);

        virtual RenderObject* createRenderer(RenderArena*, RenderStyle*);
        virtual SVGResource* canvasResource(const RenderObject*);

        PassOwnPtr<ImageBuffer> drawMaskerContent(const RenderObject*, FloatRect& maskRect, bool& emptyMask) const;

    private:
        DECLARE_ANIMATED_PROPERTY(SVGMaskElement, SVGNames::maskUnitsAttr, int, MaskUnits, maskUnits)
        DECLARE_ANIMATED_PROPERTY(SVGMaskElement, SVGNames::maskContentUnitsAttr, int, MaskContentUnits, maskContentUnits)
        DECLARE_ANIMATED_PROPERTY(SVGMaskElement, SVGNames::xAttr, SVGLength, X, x)
        DECLARE_ANIMATED_PROPERTY(SVGMaskElement, SVGNames::yAttr, SVGLength, Y, y)
        DECLARE_ANIMATED_PROPERTY(SVGMaskElement, SVGNames::widthAttr, SVGLength, Width, width)
        DECLARE_ANIMATED_PROPERTY(SVGMaskElement, SVGNames::heightAttr, SVGLength, Height, height)

        // SVGURIReference
        DECLARE_ANIMATED_PROPERTY(SVGMaskElement, XLinkNames::hrefAttr, String, Href, href)

        // SVGExternalResourcesRequired
        DECLARE_ANIMATED_PROPERTY(SVGMaskElement, SVGNames::externalResourcesRequiredAttr, bool, ExternalResourcesRequired, externalResourcesRequired)

        virtual void invalidateCanvasResources();

        HashMap<const RenderObject*, RefPtr<SVGResourceMasker> > m_masker;
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif
