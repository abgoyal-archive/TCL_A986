

#include "config.h"

#if ENABLE(FILTERS)
#include "FEGaussianBlur.h"

#include "CanvasPixelArray.h"
#include "Filter.h"
#include "GraphicsContext.h"
#include "ImageData.h"
#include <math.h>
#include <wtf/MathExtras.h>

using std::max;

namespace WebCore {

FEGaussianBlur::FEGaussianBlur(FilterEffect* in, const float& x, const float& y)
    : FilterEffect()
    , m_in(in)
    , m_x(x)
    , m_y(y)
{
}

PassRefPtr<FEGaussianBlur> FEGaussianBlur::create(FilterEffect* in, const float& x, const float& y)
{
    return adoptRef(new FEGaussianBlur(in, x, y));
}

float FEGaussianBlur::stdDeviationX() const
{
    return m_x;
}

void FEGaussianBlur::setStdDeviationX(float x)
{
    m_x = x;
}

float FEGaussianBlur::stdDeviationY() const
{
    return m_y;
}

void FEGaussianBlur::setStdDeviationY(float y)
{
    m_y = y;
}

static void boxBlur(CanvasPixelArray*& srcPixelArray, CanvasPixelArray*& dstPixelArray,
                 unsigned dx, int stride, int strideLine, int effectWidth, int effectHeight, bool alphaImage)
{
    int dxLeft = dx / 2;
    int dxRight = dx - dxLeft;

    for (int y = 0; y < effectHeight; ++y) {
        int line = y * strideLine;
        for (int channel = 3; channel >= 0; --channel) {
            int sum = 0;
            // Fill the kernel
            int maxKernelSize = std::min(dxRight, effectWidth);
            for (int i = 0; i < maxKernelSize; ++i)
                sum += srcPixelArray->get(line + i * stride + channel);

            // Blurring
            for (int x = 0; x < effectWidth; ++x) {
                int pixelByteOffset = line + x * stride + channel;
                dstPixelArray->set(pixelByteOffset, static_cast<unsigned char>(sum / dx));
                if (x >= dxLeft)
                    sum -= srcPixelArray->get(pixelByteOffset - dxLeft * stride);
                if (x + dxRight < effectWidth)
                    sum += srcPixelArray->get(pixelByteOffset + dxRight * stride);
            }
            if (alphaImage) // Source image is black, it just has different alpha values
                break;
        }
    }
}

void FEGaussianBlur::apply(Filter* filter)
{
    m_in->apply(filter);
    if (!m_in->resultImage())
        return;

    if (!getEffectContext())
        return;

    setIsAlphaImage(m_in->isAlphaImage());

    if (m_x == 0 || m_y == 0)
        return;

    unsigned sdx = static_cast<unsigned>(floor(m_x * filter->filterResolution().width() * 3 * sqrt(2 * piDouble) / 4.f + 0.5f));
    unsigned sdy = static_cast<unsigned>(floor(m_y * filter->filterResolution().height() * 3 * sqrt(2 * piDouble) / 4.f + 0.5f));
    sdx = max(sdx, static_cast<unsigned>(1));
    sdy = max(sdy, static_cast<unsigned>(1));

    IntRect effectDrawingRect = calculateDrawingIntRect(m_in->scaledSubRegion());
    RefPtr<ImageData> srcImageData(m_in->resultImage()->getPremultipliedImageData(effectDrawingRect));
    CanvasPixelArray* srcPixelArray(srcImageData->data());

    IntRect imageRect(IntPoint(), resultImage()->size());
    RefPtr<ImageData> tmpImageData = ImageData::create(imageRect.width(), imageRect.height());
    CanvasPixelArray* tmpPixelArray(tmpImageData->data());

    int stride = 4 * imageRect.width();
    for (int i = 0; i < 3; ++i) {
        boxBlur(srcPixelArray, tmpPixelArray, sdx, 4, stride, imageRect.width(), imageRect.height(), isAlphaImage());
        boxBlur(tmpPixelArray, srcPixelArray, sdy, stride, 4, imageRect.height(), imageRect.width(), isAlphaImage());
    }

    resultImage()->putPremultipliedImageData(srcImageData.get(), imageRect, IntPoint());
}

void FEGaussianBlur::dump()
{
}

} // namespace WebCore

#endif // ENABLE(FILTERS)
