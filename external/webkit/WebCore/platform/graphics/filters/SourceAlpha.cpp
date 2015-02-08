

#include "config.h"

#if ENABLE(FILTERS)
#include "SourceAlpha.h"

#include "Color.h"
#include "GraphicsContext.h"
#include "PlatformString.h"
#include "Filter.h"

#include <wtf/StdLibExtras.h>

namespace WebCore {

PassRefPtr<SourceAlpha> SourceAlpha::create()
{
    return adoptRef(new SourceAlpha);
}

const AtomicString& SourceAlpha::effectName()
{
    DEFINE_STATIC_LOCAL(const AtomicString, s_effectName, ("SourceAlpha"));
    return s_effectName;
}

FloatRect SourceAlpha::calculateEffectRect(Filter* filter)
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

void SourceAlpha::apply(Filter* filter)
{
    GraphicsContext* filterContext = getEffectContext();
    if (!filterContext)
        return;

    setIsAlphaImage(true);

    FloatRect imageRect(FloatPoint(), filter->sourceImage()->image()->size());
    filterContext->save();
    filterContext->clipToImageBuffer(imageRect, filter->sourceImage());
    filterContext->fillRect(imageRect, Color::black, DeviceColorSpace);
    filterContext->restore();
}

void SourceAlpha::dump()
{
}

} // namespace WebCore

#endif // ENABLE(FILTERS)
