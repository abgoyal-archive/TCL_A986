

#ifndef SVGResourceMasker_h
#define SVGResourceMasker_h

#if ENABLE(SVG)

#include "GraphicsContext.h"
#include "RenderObject.h"
#include "SVGMaskElement.h"
#include "SVGResource.h"

#include <memory>

#include <wtf/OwnPtr.h>
#include <wtf/PassRefPtr.h>

namespace WebCore {

    class FloatRect;
    class ImageBuffer;
    class SVGMaskElement;

    class SVGResourceMasker : public SVGResource {
    public:
        static PassRefPtr<SVGResourceMasker> create(const SVGMaskElement* ownerElement) { return adoptRef(new SVGResourceMasker(ownerElement)); }
        virtual ~SVGResourceMasker();
        
        virtual void invalidate();
        
        virtual SVGResourceType resourceType() const { return MaskerResourceType; }
        virtual TextStream& externalRepresentation(TextStream&) const;

        FloatRect maskerBoundingBox(const FloatRect&) const;
        bool applyMask(GraphicsContext*, const RenderObject*);

    private:
        SVGResourceMasker(const SVGMaskElement*);

        const SVGMaskElement* m_ownerElement;
        
        OwnPtr<ImageBuffer> m_mask;
        FloatRect m_maskRect;
        bool m_emptyMask;
    };

    SVGResourceMasker* getMaskerById(Document*, const AtomicString&, const RenderObject* object);

} // namespace WebCore

#endif

#endif // SVGResourceMasker_h
