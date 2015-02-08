

#ifndef SVGFilter_h
#define SVGFilter_h

#if ENABLE(SVG) && ENABLE(FILTERS)
#include "Filter.h"
#include "FilterEffect.h"
#include "FloatRect.h"
#include "FloatSize.h"

#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>
#include <wtf/RefPtr.h>

namespace WebCore {

    class SVGFilter : public Filter {
    public:
        static PassRefPtr<SVGFilter> create(const FloatRect&, const FloatRect&, bool);

        virtual bool effectBoundingBoxMode() const { return m_effectBBoxMode; }

        virtual FloatRect filterRegion() const { return m_filterRect; }
        virtual FloatRect sourceImageRect() const { return m_itemBox; }

        virtual FloatSize maxImageSize() const { return m_maxImageSize; }
        virtual void calculateEffectSubRegion(FilterEffect*);

    private:
        SVGFilter(const FloatRect& itemBox, const FloatRect& filterRect, bool effectBBoxMode);

        FloatSize m_maxImageSize;
        FloatRect m_itemBox;
        FloatRect m_filterRect;
        bool m_effectBBoxMode;
    };

} // namespace WebCore

#endif // ENABLE(SVG) && ENABLE(FILTERS)

#endif // SVGFilter_h
