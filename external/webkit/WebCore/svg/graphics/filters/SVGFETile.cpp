

#include "config.h"

#if ENABLE(SVG) && ENABLE(FILTERS)
#include "SVGFETile.h"

#include "AffineTransform.h"
#include "Filter.h"
#include "GraphicsContext.h"
#include "Pattern.h"
#include "SVGRenderTreeAsText.h"

namespace WebCore {

FETile::FETile(FilterEffect* in)
    : FilterEffect()
    , m_in(in)
{
}

PassRefPtr<FETile> FETile::create(FilterEffect* in)
{
    return adoptRef(new FETile(in));
}

FloatRect FETile::uniteChildEffectSubregions(Filter* filter)
{
    m_in->calculateEffectRect(filter);
    return filter->filterRegion();
}

void FETile::apply(Filter* filter)
{
    m_in->apply(filter);
    if (!m_in->resultImage())
        return;

    GraphicsContext* filterContext = getEffectContext();
    if (!filterContext)
        return;

    setIsAlphaImage(m_in->isAlphaImage());

    IntRect tileRect = enclosingIntRect(m_in->scaledSubRegion());

    // Source input needs more attention. It has the size of the filterRegion but gives the
    // size of the cutted sourceImage back. This is part of the specification and optimization.
    if (m_in->isSourceInput()) {
        FloatRect filterRegion = filter->filterRegion();
        filterRegion.scale(filter->filterResolution().width(), filter->filterResolution().height());
        tileRect = enclosingIntRect(filterRegion);
    }

    OwnPtr<ImageBuffer> tileImage = ImageBuffer::create(tileRect.size());
    GraphicsContext* tileImageContext = tileImage->context();
    tileImageContext->drawImage(m_in->resultImage()->image(), DeviceColorSpace, IntPoint());
    RefPtr<Pattern> pattern = Pattern::create(tileImage->image(), true, true);

    AffineTransform matrix;
    matrix.translate(m_in->scaledSubRegion().x() - scaledSubRegion().x(), m_in->scaledSubRegion().y() - scaledSubRegion().y());
    pattern.get()->setPatternSpaceTransform(matrix);

    filterContext->setFillPattern(pattern);
    filterContext->fillRect(FloatRect(FloatPoint(), scaledSubRegion().size()));
}

void FETile::dump()
{
}

TextStream& FETile::externalRepresentation(TextStream& ts) const
{
    ts << "[type=TILE]";
    FilterEffect::externalRepresentation(ts);
    return ts;
}

} // namespace WebCore

#endif // ENABLE(SVG) && ENABLE(FILTERS)

