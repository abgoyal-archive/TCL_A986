

#ifndef ImageBufferFilter_h
#define ImageBufferFilter_h

#if ENABLE(FILTERS)
#include "Filter.h"
#include "FilterEffect.h"
#include "FloatRect.h"
#include "FloatSize.h"

#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>
#include <wtf/RefPtr.h>

namespace WebCore {

class ImageBufferFilter : public Filter {
public:
    static PassRefPtr<ImageBufferFilter> create();

    virtual FloatRect filterRegion() const { return FloatRect(); }
    virtual FloatRect sourceImageRect() const { return FloatRect(); }

    // SVG specific
    virtual bool effectBoundingBoxMode() const { return false; }

    virtual FloatSize maxImageSize() const { return FloatSize(); }
    virtual void calculateEffectSubRegion(FilterEffect*) { }

private:
    ImageBufferFilter();
};

} // namespace WebCore

#endif // ENABLE(FILTERS)

#endif // ImageBufferFilter_h
