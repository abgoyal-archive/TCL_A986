

#ifndef GeneratedImage_h
#define GeneratedImage_h

#include "Image.h"

#include "Generator.h"
#include "IntSize.h"
#include <wtf/RefPtr.h>

namespace WebCore {

class GeneratedImage : public Image {
public:
    static PassRefPtr<GeneratedImage> create(PassRefPtr<Generator> generator, const IntSize& size)
    {
        return adoptRef(new GeneratedImage(generator, size));
    }
    virtual ~GeneratedImage() {}

    virtual bool hasSingleSecurityOrigin() const { return true; }

    // These are only used for SVGGeneratedImage right now
    virtual void setContainerSize(const IntSize& size) { m_size = size; }
    virtual bool usesContainerSize() const { return true; }
    virtual bool hasRelativeWidth() const { return true; }
    virtual bool hasRelativeHeight() const { return true; }

    virtual IntSize size() const { return m_size; }

    // Assume that generated content has no decoded data we need to worry about
    virtual void destroyDecodedData(bool /*destroyAll*/ = true) { }
    virtual unsigned decodedSize() const { return 0; }

protected:
    virtual void draw(GraphicsContext*, const FloatRect& dstRect, const FloatRect& srcRect, ColorSpace styleColorSpace, CompositeOperator);
    virtual void drawPattern(GraphicsContext*, const FloatRect& srcRect, const AffineTransform& patternTransform,
                             const FloatPoint& phase, ColorSpace styleColorSpace, CompositeOperator, const FloatRect& destRect);
    
    GeneratedImage(PassRefPtr<Generator> generator, const IntSize& size)
        : m_generator(generator)
        , m_size(size)
    {
    }

    RefPtr<Generator> m_generator;
    IntSize m_size;
};

}

#endif
