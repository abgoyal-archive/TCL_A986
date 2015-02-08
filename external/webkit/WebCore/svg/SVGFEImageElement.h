

#ifndef SVGFEImageElement_h
#define SVGFEImageElement_h

#if ENABLE(SVG) && ENABLE(FILTERS)
#include "CachedResourceHandle.h"
#include "ImageBuffer.h"
#include "SVGExternalResourcesRequired.h"
#include "SVGFEImage.h"
#include "SVGFilterPrimitiveStandardAttributes.h"
#include "SVGLangSpace.h"
#include "SVGPreserveAspectRatio.h"
#include "SVGURIReference.h"

namespace WebCore {

    class SVGFEImageElement : public SVGFilterPrimitiveStandardAttributes,
                              public SVGURIReference,
                              public SVGLangSpace,
                              public SVGExternalResourcesRequired,
                              public CachedResourceClient {
    public:
        SVGFEImageElement(const QualifiedName&, Document*);
        virtual ~SVGFEImageElement();

        virtual void parseMappedAttribute(MappedAttribute*);
        virtual void synchronizeProperty(const QualifiedName&);
        virtual void notifyFinished(CachedResource*);

        virtual void addSubresourceAttributeURLs(ListHashSet<KURL>&) const;
        virtual bool build(SVGResourceFilter*);

    private:
        void requestImageResource();

        DECLARE_ANIMATED_PROPERTY(SVGFEImageElement, SVGNames::preserveAspectRatioAttr, SVGPreserveAspectRatio, PreserveAspectRatio, preserveAspectRatio)

        // SVGURIReference
        DECLARE_ANIMATED_PROPERTY(SVGFEImageElement, XLinkNames::hrefAttr, String, Href, href)

        // SVGExternalResourcesRequired
        DECLARE_ANIMATED_PROPERTY(SVGFEImageElement, SVGNames::externalResourcesRequiredAttr, bool, ExternalResourcesRequired, externalResourcesRequired)

        CachedResourceHandle<CachedImage> m_cachedImage;
        OwnPtr<ImageBuffer> m_targetImage;
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif
