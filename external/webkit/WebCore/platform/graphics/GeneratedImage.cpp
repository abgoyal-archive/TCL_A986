

#include "config.h"
#include "GeneratedImage.h"

#include "FloatRect.h"
#include "GraphicsContext.h"
#include "ImageBuffer.h"

using namespace std;

namespace WebCore {

void GeneratedImage::draw(GraphicsContext* context, const FloatRect& dstRect, const FloatRect& srcRect, ColorSpace, CompositeOperator compositeOp)
{
    context->save();
    context->setCompositeOperation(compositeOp);
    context->clip(dstRect);
    context->translate(dstRect.x(), dstRect.y());
    if (dstRect.size() != srcRect.size())
        context->scale(FloatSize(dstRect.width() / srcRect.width(), dstRect.height() / srcRect.height()));
    context->translate(-srcRect.x(), -srcRect.y());
    context->fillRect(FloatRect(FloatPoint(), m_size), *m_generator.get());
    context->restore();
}

void GeneratedImage::drawPattern(GraphicsContext* context, const FloatRect& srcRect, const AffineTransform& patternTransform,
                                 const FloatPoint& phase, ColorSpace styleColorSpace, CompositeOperator compositeOp, const FloatRect& destRect)
{
    // Allow the generator to provide visually-equivalent tiling parameters for better performance.
    IntSize adjustedSize = m_size;
    FloatRect adjustedSrcRect = srcRect;
    m_generator->adjustParametersForTiledDrawing(adjustedSize, adjustedSrcRect);

    // Create a BitmapImage and call drawPattern on it.
    OwnPtr<ImageBuffer> imageBuffer = ImageBuffer::create(adjustedSize);
    ASSERT(imageBuffer.get());

    // Fill with the gradient.
    GraphicsContext* graphicsContext = imageBuffer->context();
    graphicsContext->fillRect(FloatRect(FloatPoint(), adjustedSize), *m_generator.get());

    // Grab the final image from the image buffer.
    Image* bitmap = imageBuffer->image();

    // Now just call drawTiled on that image.
    bitmap->drawPattern(context, adjustedSrcRect, patternTransform, phase, styleColorSpace, compositeOp, destRect);
}

}
