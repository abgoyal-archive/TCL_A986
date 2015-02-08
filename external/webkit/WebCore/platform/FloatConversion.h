

#ifndef FloatConversion_h
#define FloatConversion_h

#include <wtf/Platform.h>
#if PLATFORM(CG)
#include <CoreGraphics/CGBase.h>
#endif

namespace WebCore {

    template<typename T>
    float narrowPrecisionToFloat(T);

    template<>
    inline float narrowPrecisionToFloat(double number)
    {
        return static_cast<float>(number);
    }

#if PLATFORM(CG)
    template<typename T>
    CGFloat narrowPrecisionToCGFloat(T);

    template<>
    inline CGFloat narrowPrecisionToCGFloat(double number)
    {
        return static_cast<CGFloat>(number);
    }
#endif

} // namespace WebCore

#endif // FloatConversion_h
