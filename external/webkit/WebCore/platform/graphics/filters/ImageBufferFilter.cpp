

#include "config.h"

#if ENABLE(FILTERS)
#include "ImageBufferFilter.h"

#include "FloatSize.h"

namespace WebCore {

ImageBufferFilter::ImageBufferFilter()
    : Filter()
{
    setFilterResolution(FloatSize(1.f, 1.f));
}

PassRefPtr<ImageBufferFilter> ImageBufferFilter::create()
{
    return adoptRef(new ImageBufferFilter());
}

} // namespace WebCore

#endif // ENABLE(FILTERS)
