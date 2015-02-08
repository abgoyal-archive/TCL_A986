

#ifndef SVGStyledElement_h
#define SVGStyledElement_h

#if ENABLE(SVG)
#include "HTMLNames.h"
#include "SVGElement.h"
#include "SVGStylable.h"

namespace WebCore {

    class SVGResource;

    void mapAttributeToCSSProperty(HashMap<AtomicStringImpl*, int>* propertyNameToIdMap, const QualifiedName& attrName);

    class SVGStyledElement : public SVGElement,
                             public SVGStylable {
    public:
        SVGStyledElement(const QualifiedName&, Document*);
        virtual ~SVGStyledElement();

        virtual bool hasRelativeValues() const { return false; }
        virtual bool isStyled() const { return true; }
        virtual bool supportsMarkers() const { return false; }

        virtual PassRefPtr<CSSValue> getPresentationAttribute(const String& name);
        virtual CSSStyleDeclaration* style() { return StyledElement::style(); }

        bool isKnownAttribute(const QualifiedName&);

        virtual bool rendererIsNeeded(RenderStyle*);
        virtual SVGResource* canvasResource(const RenderObject*) { return 0; }

        virtual bool mapToEntry(const QualifiedName&, MappedAttributeEntry&) const;
        virtual void parseMappedAttribute(MappedAttribute*);
        virtual void svgAttributeChanged(const QualifiedName&);
        virtual void synchronizeProperty(const QualifiedName&);

        virtual void childrenChanged(bool changedByParser = false, Node* beforeChange = 0, Node* afterChange = 0, int childCountDelta = 0);

        // Centralized place to force a manual style resolution. Hacky but needed for now.
        PassRefPtr<RenderStyle> resolveStyle(RenderStyle* parentStyle);

        void invalidateResourcesInAncestorChain() const;
        void invalidateResources();

        virtual void detach();

        bool instanceUpdatesBlocked() const;
        void setInstanceUpdatesBlocked(bool);

    protected: 
        static int cssPropertyIdForSVGAttributeName(const QualifiedName&);

        virtual void invalidateCanvasResources();

    private:
        DECLARE_ANIMATED_PROPERTY(SVGStyledElement, HTMLNames::classAttr, String, ClassName, className)
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif // SVGStyledElement
