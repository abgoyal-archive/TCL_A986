

#include "config.h"
#include "FloatSize.h"

#include "FloatConversion.h"
#include "IntSize.h"

namespace WebCore {

FloatSize::FloatSize(const IntSize& size) : m_width(size.width()), m_height(size.height())
{
}

FloatSize FloatSize::narrowPrecision(double width, double height)
{
    return FloatSize(narrowPrecisionToFloat(width), narrowPrecisionToFloat(height));
}

}
