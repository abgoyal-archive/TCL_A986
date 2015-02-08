

#ifndef SVGTRefElement_h
#define SVGTRefElement_h

#if ENABLE(SVG)
#include "SVGTextPositioningElement.h"
#include "SVGURIReference.h"

namespace WebCore {

    class SVGTRefElement : public SVGTextPositioningElement,
                           public SVGURIReference {
    public:
        SVGTRefElement(const QualifiedName&, Document*);
        virtual ~SVGTRefElement();

        virtual void parseMappedAttribute(MappedAttribute*);
        virtual void svgAttributeChanged(const QualifiedName&);
        virtual void synchronizeProperty(const QualifiedName&);

        virtual RenderObject* createRenderer(RenderArena*, RenderStyle*);
        bool childShouldCreateRenderer(Node*) const;

    private:
        void updateReferencedText();

        // SVGURIReference
        DECLARE_ANIMATED_PROPERTY(SVGTRefElement, XLinkNames::hrefAttr, String, Href, href)
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif
