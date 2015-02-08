

#ifndef Complex_h
#define Complex_h

#include <complex>
#include <wtf/MathExtras.h>

namespace WebCore {

typedef std::complex<double> Complex;

inline Complex complexFromMagnitudePhase(double magnitude, double phase)
{
    return Complex(magnitude * cos(phase), magnitude * sin(phase));
}

} // namespace WebCore

#endif // Complex_h
