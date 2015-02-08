

#ifndef SVGUseElement_h
#define SVGUseElement_h

#if ENABLE(SVG)
#include "SVGExternalResourcesRequired.h"
#include "SVGLangSpace.h"
#include "SVGStyledTransformableElement.h"
#include "SVGTests.h"
#include "SVGURIReference.h"

namespace WebCore {

    class SVGElementInstance;
    class SVGLength;
    class SVGShadowTreeRootElement;

    class SVGUseElement : public SVGStyledTransformableElement,
                          public SVGTests,
                          public SVGLangSpace,
                          public SVGExternalResourcesRequired,
                          public SVGURIReference {
    public:
        SVGUseElement(const QualifiedName&, Document*);
        virtual ~SVGUseElement();

        SVGElementInstance* instanceRoot() const;
        SVGElementInstance* animatedInstanceRoot() const;

        virtual bool isValid() const { return SVGTests::isValid(); }

        virtual void insertedIntoDocument();
        virtual void removedFromDocument();
        virtual void buildPendingResource();

        virtual void parseMappedAttribute(MappedAttribute*);
        virtual void svgAttributeChanged(const QualifiedName&);
        virtual void synchronizeProperty(const QualifiedName&);

        virtual void recalcStyle(StyleChange = NoChange);
        virtual RenderObject* createRenderer(RenderArena*, RenderStyle*);
        virtual void attach();
        virtual void detach();

        virtual Path toClipPath() const;

        static void removeDisallowedElementsFromSubtree(Node* element);
        SVGElementInstance* instanceForShadowTreeElement(Node* element) const;
        void invalidateShadowTree();

    private:
        friend class RenderSVGShadowTreeRootContainer;
        bool isPendingResource() const { return m_isPendingResource; }
        void buildShadowAndInstanceTree(SVGShadowTreeRootElement*);

    private:
        virtual bool hasRelativeValues() const;

        DECLARE_ANIMATED_PROPERTY(SVGUseElement, SVGNames::xAttr, SVGLength, X, x)
        DECLARE_ANIMATED_PROPERTY(SVGUseElement, SVGNames::yAttr, SVGLength, Y, y)
        DECLARE_ANIMATED_PROPERTY(SVGUseElement, SVGNames::widthAttr, SVGLength, Width, width)
        DECLARE_ANIMATED_PROPERTY(SVGUseElement, SVGNames::heightAttr, SVGLength, Height, height)

        // SVGURIReference
        DECLARE_ANIMATED_PROPERTY(SVGUseElement, XLinkNames::hrefAttr, String, Href, href)

        // SVGExternalResourcesRequired
        DECLARE_ANIMATED_PROPERTY(SVGUseElement, SVGNames::externalResourcesRequiredAttr, bool, ExternalResourcesRequired, externalResourcesRequired)

    private:
        // Instance tree handling
        void buildInstanceTree(SVGElement* target, SVGElementInstance* targetInstance, bool& foundCycle);
        void handleDeepUseReferencing(SVGUseElement* use, SVGElementInstance* targetInstance, bool& foundCycle);

        // Shadow tree handling
        void buildShadowTree(SVGShadowTreeRootElement*, SVGElement* target, SVGElementInstance* targetInstance);

#if ENABLE(SVG) && ENABLE(SVG_USE)
        void expandUseElementsInShadowTree(SVGShadowTreeRootElement*, Node* element);
        void expandSymbolElementsInShadowTree(SVGShadowTreeRootElement*, Node* element);
#endif

        // "Tree connector" 
        void associateInstancesWithShadowTreeElements(Node* target, SVGElementInstance* targetInstance);
        SVGElementInstance* instanceForShadowTreeElement(Node* element, SVGElementInstance* instance) const;

        void transferUseAttributesToReplacedElement(SVGElement* from, SVGElement* to) const;
        void transferEventListenersToShadowTree(SVGElementInstance* target);

        void updateContainerOffsets();
        void updateContainerSizes();

        bool m_isPendingResource;
        bool m_needsShadowTreeRecreation;
        String m_resourceId;
        RefPtr<SVGElementInstance> m_targetElementInstance;
    };

}

#endif
#endif
