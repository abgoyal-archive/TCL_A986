

#ifndef SVGElement_h
#define SVGElement_h

#if ENABLE(SVG)
#include "SVGDocumentExtensions.h"
#include "StyledElement.h"

namespace WebCore {

    class CSSCursorImageValue;
    class Document;
    class SVGCursorElement;
    class SVGElementInstance;
    class SVGElementRareData;
    class SVGSVGElement;
    class AffineTransform;

    class SVGElement : public StyledElement {
    public:
        static PassRefPtr<SVGElement> create(const QualifiedName&, Document*);
        virtual ~SVGElement();

        String xmlbase() const;
        void setXmlbase(const String&, ExceptionCode&);

        SVGSVGElement* ownerSVGElement() const;
        SVGElement* viewportElement() const;

        SVGDocumentExtensions* accessDocumentSVGExtensions() const;

        virtual void parseMappedAttribute(MappedAttribute*);

        virtual bool isStyled() const { return false; }
        virtual bool isStyledTransformable() const { return false; }
        virtual bool isStyledLocatable() const { return false; }
        virtual bool isSVG() const { return false; }
        virtual bool isFilterEffect() const { return false; }
        virtual bool isGradientStop() const { return false; }
        virtual bool isTextContent() const { return false; }

        // For SVGTests
        virtual bool isValid() const { return true; }

        virtual bool rendererIsNeeded(RenderStyle*) { return false; }
        virtual bool childShouldCreateRenderer(Node*) const;

        virtual void svgAttributeChanged(const QualifiedName&) { }
        virtual void synchronizeProperty(const QualifiedName&) { }

        void sendSVGLoadEventIfPossible(bool sendParentLoadEvents = false);
        
        virtual AffineTransform* supplementalTransform() { return 0; }

        void setSynchronizedSVGAttributes(bool value) { m_areSVGAttributesValid = value; }

        const HashSet<SVGElementInstance*>& instancesForElement() const;

        void setCursorElement(SVGCursorElement*);
        void setCursorImageValue(CSSCursorImageValue*);

    protected:
        SVGElement(const QualifiedName&, Document*);

        virtual void finishParsingChildren();
        virtual void insertedIntoDocument();
        virtual void attributeChanged(Attribute*, bool preserveDecls = false);
        virtual void updateAnimatedSVGAttribute(const QualifiedName&) const;

        SVGElementRareData* rareSVGData() const;
        SVGElementRareData* ensureRareSVGData();

    private:
        friend class SVGElementInstance;

        virtual bool isSVGElement() const { return true; }
        virtual bool isSupported(StringImpl* feature, StringImpl* version) const;

        virtual ContainerNode* eventParentNode();
        virtual void buildPendingResource() { }

        void mapInstanceToElement(SVGElementInstance*);
        void removeInstanceMapping(SVGElementInstance*);

        virtual bool haveLoadedRequiredResources();
    };

}

// This file needs to be included after the SVGElement declaration
#include "SVGAnimatedProperty.h"

#endif
#endif
