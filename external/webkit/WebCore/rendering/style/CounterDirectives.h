

#ifndef CounterDirectives_h
#define CounterDirectives_h

#include "AtomicStringImpl.h"
#include <wtf/HashMap.h>
#include <wtf/RefPtr.h>

namespace WebCore {

struct CounterDirectives {
    CounterDirectives()
        : m_reset(false)
        , m_increment(false)
    {
    }

    bool m_reset;
    int m_resetValue;
    bool m_increment;
    int m_incrementValue;
};

bool operator==(const CounterDirectives&, const CounterDirectives&);
inline bool operator!=(const CounterDirectives& a, const CounterDirectives& b) { return !(a == b); }

typedef HashMap<RefPtr<AtomicStringImpl>, CounterDirectives> CounterDirectiveMap;

} // namespace WebCore

#endif // CounterDirectives_h
