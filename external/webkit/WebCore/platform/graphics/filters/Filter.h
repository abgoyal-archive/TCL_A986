

#ifndef Filter_h
#define Filter_h

#if ENABLE(FILTERS)
#include "FloatRect.h"
#include "FloatSize.h"
#include "ImageBuffer.h"
#include "StringHash.h"

#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>
#include <wtf/RefPtr.h>

namespace WebCore {

    class FilterEffect;

    class Filter : public RefCounted<Filter> {
    public:
        virtual ~Filter() { }

        void setSourceImage(PassOwnPtr<ImageBuffer> sourceImage) { m_sourceImage = sourceImage; }
        ImageBuffer* sourceImage() { return m_sourceImage.get(); }

        FloatSize filterResolution() const { return m_filterResolution; }
        void setFilterResolution(const FloatSize& filterResolution) { m_filterResolution = filterResolution; }

        virtual FloatRect sourceImageRect() const = 0;
        virtual FloatRect filterRegion() const = 0;

        // SVG specific
        virtual void calculateEffectSubRegion(FilterEffect*) { }

        virtual FloatSize maxImageSize() const = 0;
        virtual bool effectBoundingBoxMode() const = 0;

    private:
        OwnPtr<ImageBuffer> m_sourceImage;
        FloatSize m_filterResolution;
    };

} // namespace WebCore

#endif // ENABLE(FILTERS)

#endif // Filter_h
