

#ifndef SVGClipPathElement_h
#define SVGClipPathElement_h

#if ENABLE(SVG)
#include "RenderObject.h"
#include "SVGExternalResourcesRequired.h"
#include "SVGLangSpace.h"
#include "SVGResourceClipper.h"
#include "SVGStyledTransformableElement.h"
#include "SVGTests.h"

namespace WebCore {

    class SVGClipPathElement : public SVGStyledTransformableElement,
                               public SVGTests,
                               public SVGLangSpace,
                               public SVGExternalResourcesRequired {
    public:
        SVGClipPathElement(const QualifiedName&, Document*);
        virtual ~SVGClipPathElement();

        virtual bool isValid() const { return SVGTests::isValid(); }

        virtual void parseMappedAttribute(MappedAttribute*);
        virtual void svgAttributeChanged(const QualifiedName&);
        virtual void synchronizeProperty(const QualifiedName&);
        virtual void childrenChanged(bool changedByParser = false, Node* beforeChange = 0, Node* afterChange = 0, int childCountDelta = 0);

        virtual RenderObject* createRenderer(RenderArena*, RenderStyle*);
        virtual SVGResource* canvasResource(const RenderObject*);

    private:
        DECLARE_ANIMATED_PROPERTY(SVGClipPathElement, SVGNames::clipPathUnitsAttr, int, ClipPathUnits, clipPathUnits)

        // SVGExternalResourcesRequired
        DECLARE_ANIMATED_PROPERTY(SVGClipPathElement, SVGNames::externalResourcesRequiredAttr, bool, ExternalResourcesRequired, externalResourcesRequired)

        RefPtr<SVGResourceClipper> m_clipper;
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif
