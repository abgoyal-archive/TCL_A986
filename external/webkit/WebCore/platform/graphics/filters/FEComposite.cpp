

#include "config.h"

#if ENABLE(FILTERS)
#include "FEComposite.h"

#include "CanvasPixelArray.h"
#include "Filter.h"
#include "GraphicsContext.h"
#include "ImageData.h"

namespace WebCore {

FEComposite::FEComposite(FilterEffect* in, FilterEffect* in2, const CompositeOperationType& type,
    const float& k1, const float& k2, const float& k3, const float& k4)
    : FilterEffect()
    , m_in(in)
    , m_in2(in2)
    , m_type(type)
    , m_k1(k1)
    , m_k2(k2)
    , m_k3(k3)
    , m_k4(k4)
{
}

PassRefPtr<FEComposite> FEComposite::create(FilterEffect* in, FilterEffect* in2, const CompositeOperationType& type,
    const float& k1, const float& k2, const float& k3, const float& k4)
{
    return adoptRef(new FEComposite(in, in2, type, k1, k2, k3, k4));
}

CompositeOperationType FEComposite::operation() const
{
    return m_type;
}

void FEComposite::setOperation(CompositeOperationType type)
{
    m_type = type;
}

float FEComposite::k1() const
{
    return m_k1;
}

void FEComposite::setK1(float k1)
{
    m_k1 = k1;
}

float FEComposite::k2() const
{
    return m_k2;
}

void FEComposite::setK2(float k2)
{
    m_k2 = k2;
}

float FEComposite::k3() const
{
    return m_k3;
}

void FEComposite::setK3(float k3)
{
    m_k3 = k3;
}

float FEComposite::k4() const
{
    return m_k4;
}

void FEComposite::setK4(float k4)
{
    m_k4 = k4;
}

inline void arithmetic(const RefPtr<CanvasPixelArray>& srcPixelArrayA, CanvasPixelArray*& srcPixelArrayB,
                       float k1, float k2, float k3, float k4)
{
    float scaledK1 = k1 / 255.f;
    float scaledK4 = k4 * 255.f;
    for (unsigned pixelOffset = 0; pixelOffset < srcPixelArrayA->length(); pixelOffset += 4) {
        for (unsigned channel = 0; channel < 4; ++channel) {
            unsigned char i1 = srcPixelArrayA->get(pixelOffset + channel);
            unsigned char i2 = srcPixelArrayB->get(pixelOffset + channel);

            double result = scaledK1 * i1 * i2 + k2 * i1 + k3 * i2 + scaledK4;
            srcPixelArrayB->set(pixelOffset + channel, result);
        }
    }
}

void FEComposite::apply(Filter* filter)
{
    m_in->apply(filter);
    m_in2->apply(filter);
    if (!m_in->resultImage() || !m_in2->resultImage())
        return;

    GraphicsContext* filterContext = getEffectContext();
    if (!filterContext)
        return;

    FloatRect srcRect = FloatRect(0.f, 0.f, -1.f, -1.f);
    switch (m_type) {
    case FECOMPOSITE_OPERATOR_OVER:
        filterContext->drawImage(m_in2->resultImage()->image(), DeviceColorSpace, calculateDrawingRect(m_in2->scaledSubRegion()));
        filterContext->drawImage(m_in->resultImage()->image(), DeviceColorSpace, calculateDrawingRect(m_in->scaledSubRegion()));
        break;
    case FECOMPOSITE_OPERATOR_IN:
        filterContext->save();
        filterContext->clipToImageBuffer(calculateDrawingRect(m_in2->scaledSubRegion()), m_in2->resultImage());
        filterContext->drawImage(m_in->resultImage()->image(), DeviceColorSpace, calculateDrawingRect(m_in->scaledSubRegion()));
        filterContext->restore();
        break;
    case FECOMPOSITE_OPERATOR_OUT:
        filterContext->drawImage(m_in->resultImage()->image(), DeviceColorSpace, calculateDrawingRect(m_in->scaledSubRegion()));
        filterContext->drawImage(m_in2->resultImage()->image(), DeviceColorSpace, calculateDrawingRect(m_in2->scaledSubRegion()), srcRect, CompositeDestinationOut);
        break;
    case FECOMPOSITE_OPERATOR_ATOP:
        filterContext->drawImage(m_in2->resultImage()->image(), DeviceColorSpace, calculateDrawingRect(m_in2->scaledSubRegion()));
        filterContext->drawImage(m_in->resultImage()->image(), DeviceColorSpace, calculateDrawingRect(m_in->scaledSubRegion()), srcRect, CompositeSourceAtop);
        break;
    case FECOMPOSITE_OPERATOR_XOR:
        filterContext->drawImage(m_in2->resultImage()->image(), DeviceColorSpace, calculateDrawingRect(m_in2->scaledSubRegion()));
        filterContext->drawImage(m_in->resultImage()->image(), DeviceColorSpace, calculateDrawingRect(m_in->scaledSubRegion()), srcRect, CompositeXOR);
        break;
    case FECOMPOSITE_OPERATOR_ARITHMETIC: {
        IntRect effectADrawingRect = calculateDrawingIntRect(m_in->scaledSubRegion());
        RefPtr<CanvasPixelArray> srcPixelArrayA(m_in->resultImage()->getPremultipliedImageData(effectADrawingRect)->data());

        IntRect effectBDrawingRect = calculateDrawingIntRect(m_in2->scaledSubRegion());
        RefPtr<ImageData> imageData(m_in2->resultImage()->getPremultipliedImageData(effectBDrawingRect));
        CanvasPixelArray* srcPixelArrayB(imageData->data());

        arithmetic(srcPixelArrayA, srcPixelArrayB, m_k1, m_k2, m_k3, m_k4);
        resultImage()->putPremultipliedImageData(imageData.get(), IntRect(IntPoint(), resultImage()->size()), IntPoint());
        }
        break;
    default:
        break;
    }
}

void FEComposite::dump()
{
}

} // namespace WebCore

#endif // ENABLE(FILTERS)
