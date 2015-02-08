

#ifndef WeakRandom_h
#define WeakRandom_h

#include <limits.h>
#include <wtf/StdLibExtras.h>

namespace JSC {

class WeakRandom {
public:
    WeakRandom(unsigned seed)
        : m_low(seed ^ 0x49616E42)
        , m_high(seed)
    {
    }

    double get()
    {
        return advance() / (UINT_MAX + 1.0);
    }

private:
    unsigned advance()
    {
        m_high = (m_high << 16) + (m_high >> 16);
        m_high += m_low;
        m_low += m_high;
        return m_high;
    }

    unsigned m_low;
    unsigned m_high;
};

} // namespace JSC

#endif // WeakRandom_h
