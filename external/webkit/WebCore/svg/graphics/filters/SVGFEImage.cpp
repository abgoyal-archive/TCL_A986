

#include "config.h"

#if ENABLE(SVG) && ENABLE(FILTERS)
#include "SVGFEImage.h"

#include "AffineTransform.h"
#include "Filter.h"
#include "GraphicsContext.h"
#include "SVGPreserveAspectRatio.h"
#include "SVGRenderTreeAsText.h"

namespace WebCore {

FEImage::FEImage(RefPtr<Image> image, SVGPreserveAspectRatio preserveAspectRatio)
    : FilterEffect()
    , m_image(image)
    , m_preserveAspectRatio(preserveAspectRatio)
{
}

PassRefPtr<FEImage> FEImage::create(RefPtr<Image> image, SVGPreserveAspectRatio preserveAspectRatio)
{
    return adoptRef(new FEImage(image, preserveAspectRatio));
}

void FEImage::apply(Filter*)
{
    if (!m_image.get())
        return;

    GraphicsContext* filterContext = getEffectContext();
    if (!filterContext)
        return;

    FloatRect srcRect(FloatPoint(), m_image->size());
    FloatRect destRect(FloatPoint(), subRegion().size());

    m_preserveAspectRatio.transformRect(destRect, srcRect);

    filterContext->drawImage(m_image.get(), DeviceColorSpace, destRect, srcRect);
}

void FEImage::dump()
{
}

TextStream& FEImage::externalRepresentation(TextStream& ts) const
{
    ts << "[type=IMAGE] ";
    FilterEffect::externalRepresentation(ts);
    // FIXME: should this dump also object returned by SVGFEImage::image() ?
    return ts;
}

} // namespace WebCore

#endif // ENABLE(SVG) && ENABLE(FILTERS)
