

#include "config.h"

#if ENABLE(SVG)
#include "SVGResourceMasker.h"

#include "CanvasPixelArray.h"
#include "Image.h"
#include "ImageBuffer.h"
#include "ImageData.h"
#include "GraphicsContext.h"
#include "RenderObject.h"
#include "SVGMaskElement.h"
#include "SVGRenderSupport.h"
#include "SVGRenderStyle.h"
#include "TextStream.h"

using namespace std;

namespace WebCore {

SVGResourceMasker::SVGResourceMasker(const SVGMaskElement* ownerElement)
    : SVGResource()
    , m_ownerElement(ownerElement)
    , m_emptyMask(false)
{
}

SVGResourceMasker::~SVGResourceMasker()
{
}

void SVGResourceMasker::invalidate()
{
    SVGResource::invalidate();
    m_mask.clear();
    m_emptyMask = false;
}

FloatRect SVGResourceMasker::maskerBoundingBox(const FloatRect& objectBoundingBox) const
{
    return m_ownerElement->maskBoundingBox(objectBoundingBox);
}

bool SVGResourceMasker::applyMask(GraphicsContext* context, const RenderObject* object)
{
    if (!m_mask && !m_emptyMask)
        m_mask = m_ownerElement->drawMaskerContent(object, m_maskRect, m_emptyMask);

    if (!m_mask)
        return false;

    context->clipToImageBuffer(m_maskRect, m_mask.get());
    return true;
}

TextStream& SVGResourceMasker::externalRepresentation(TextStream& ts) const
{
    ts << "[type=MASKER]";
    return ts;
}

SVGResourceMasker* getMaskerById(Document* document, const AtomicString& id, const RenderObject* object)
{
    SVGResource* resource = getResourceById(document, id, object);
    if (resource && resource->isMasker())
        return static_cast<SVGResourceMasker*>(resource);

    return 0;
}

} // namespace WebCore

#endif
