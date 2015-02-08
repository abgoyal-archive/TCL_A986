

#include "config.h"

#if ENABLE(SVG) && ENABLE(FILTERS)
#include "SVGFilter.h"

namespace WebCore {

SVGFilter::SVGFilter(const FloatRect& itemBox, const FloatRect& filterRect, bool effectBBoxMode)
    : Filter()
    , m_itemBox(itemBox)
    , m_filterRect(filterRect)
    , m_effectBBoxMode(effectBBoxMode)
{
}

void SVGFilter::calculateEffectSubRegion(FilterEffect* effect)
{
    FloatRect subRegionBBox = effect->effectBoundaries();
    FloatRect useBBox = effect->unionOfChildEffectSubregions();
    FloatRect newSubRegion = subRegionBBox;

    if (m_effectBBoxMode) {
        newSubRegion = useBBox;

        if (effect->hasX())
            newSubRegion.setX(m_itemBox.x() + subRegionBBox.x() * m_itemBox.width());

        if (effect->hasY())
            newSubRegion.setY(m_itemBox.y() + subRegionBBox.y() * m_itemBox.height());

        if (effect->hasWidth())
            newSubRegion.setWidth(subRegionBBox.width() * m_itemBox.width());

        if (effect->hasHeight())
            newSubRegion.setHeight(subRegionBBox.height() * m_itemBox.height());
    } else {
        if (!effect->hasX())
            newSubRegion.setX(useBBox.x());

        if (!effect->hasY())
            newSubRegion.setY(useBBox.y());

        if (!effect->hasWidth())
            newSubRegion.setWidth(useBBox.width());

        if (!effect->hasHeight())
            newSubRegion.setHeight(useBBox.height());
    }

    // clip every filter effect to the filter region
    newSubRegion.intersect(m_filterRect);

    effect->setSubRegion(newSubRegion);
    newSubRegion.scale(filterResolution().width(), filterResolution().height());
    effect->setScaledSubRegion(newSubRegion);
    m_maxImageSize = m_maxImageSize.expandedTo(newSubRegion.size()); 
}

PassRefPtr<SVGFilter> SVGFilter::create(const FloatRect& itemBox, const FloatRect& filterRect, bool effectBBoxMode)
{
    return adoptRef(new SVGFilter(itemBox, filterRect, effectBBoxMode));
}

} // namespace WebCore

#endif // ENABLE(SVG) && ENABLE(FILTERS)
