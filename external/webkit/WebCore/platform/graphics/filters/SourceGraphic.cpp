

#include "config.h"

#if ENABLE(FILTERS)
#include "SourceGraphic.h"

#include "GraphicsContext.h"
#include "PlatformString.h"
#include "Filter.h"

#include <wtf/StdLibExtras.h>

namespace WebCore {

PassRefPtr<SourceGraphic> SourceGraphic::create()
{
    return adoptRef(new SourceGraphic);
}

const AtomicString& SourceGraphic::effectName()
{
    DEFINE_STATIC_LOCAL(const AtomicString, s_effectName, ("SourceGraphic"));
    return s_effectName;
}

FloatRect SourceGraphic::calculateEffectRect(Filter* filter)
{
    FloatRect clippedSourceRect = filter->sourceImageRect();
    if (filter->sourceImageRect().x() < filter->filterRegion().x())
        clippedSourceRect.setX(filter->filterRegion().x());
    if (filter->sourceImageRect().y() < filter->filterRegion().y())
        clippedSourceRect.setY(filter->filterRegion().y());
    setSubRegion(clippedSourceRect);
    clippedSourceRect.scale(filter->filterResolution().width(), filter->filterResolution().height());
    setScaledSubRegion(clippedSourceRect);
    return filter->filterRegion();
}

void SourceGraphic::apply(Filter* filter)
{
    GraphicsContext* filterContext = getEffectContext();
    if (!filterContext)
        return;

    filterContext->drawImage(filter->sourceImage()->image(), DeviceColorSpace, IntPoint());
}

void SourceGraphic::dump()
{
}

} // namespace WebCore

#endif // ENABLE(FILTERS)
