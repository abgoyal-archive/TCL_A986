

#ifndef SVGImageElement_h
#define SVGImageElement_h

#if ENABLE(SVG)
#include "SVGExternalResourcesRequired.h"
#include "SVGLangSpace.h"
#include "SVGImageLoader.h"
#include "SVGStyledTransformableElement.h"
#include "SVGTests.h"
#include "SVGURIReference.h"
#include "SVGPreserveAspectRatio.h"

namespace WebCore {

    class SVGLength;

    class SVGImageElement : public SVGStyledTransformableElement,
                            public SVGTests,
                            public SVGLangSpace,
                            public SVGExternalResourcesRequired,
                            public SVGURIReference {
    public:
        SVGImageElement(const QualifiedName&, Document*);
        virtual ~SVGImageElement();
        
        virtual bool isValid() const { return SVGTests::isValid(); }

        virtual void parseMappedAttribute(MappedAttribute*);
        virtual void svgAttributeChanged(const QualifiedName&);
        virtual void synchronizeProperty(const QualifiedName&);

        virtual void attach();
        virtual void insertedIntoDocument();

        virtual RenderObject* createRenderer(RenderArena*, RenderStyle*);
 
        virtual const QualifiedName& imageSourceAttributeName() const;       
        virtual void addSubresourceAttributeURLs(ListHashSet<KURL>&) const;

    protected:
        virtual bool haveLoadedRequiredResources();
        virtual bool hasRelativeValues() const;

    private:
        DECLARE_ANIMATED_PROPERTY(SVGImageElement, SVGNames::xAttr, SVGLength, X, x)
        DECLARE_ANIMATED_PROPERTY(SVGImageElement, SVGNames::yAttr, SVGLength, Y, y)
        DECLARE_ANIMATED_PROPERTY(SVGImageElement, SVGNames::widthAttr, SVGLength, Width, width)
        DECLARE_ANIMATED_PROPERTY(SVGImageElement, SVGNames::heightAttr, SVGLength, Height, height)
        DECLARE_ANIMATED_PROPERTY(SVGImageElement, SVGNames::preserveAspectRatioAttr, SVGPreserveAspectRatio, PreserveAspectRatio, preserveAspectRatio)

        // SVGURIReference
        DECLARE_ANIMATED_PROPERTY(SVGImageElement, XLinkNames::hrefAttr, String, Href, href)

        // SVGExternalResourcesRequired
        DECLARE_ANIMATED_PROPERTY(SVGImageElement, SVGNames::externalResourcesRequiredAttr, bool, ExternalResourcesRequired, externalResourcesRequired)

        SVGImageLoader m_imageLoader;
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif
