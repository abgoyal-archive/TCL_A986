

#ifndef CounterContent_h
#define CounterContent_h

#include "AtomicString.h"
#include "RenderStyleConstants.h"

namespace WebCore {

class CounterContent : public FastAllocBase {
public:
    CounterContent(const AtomicString& identifier, EListStyleType style, const AtomicString& separator)
        : m_identifier(identifier)
        , m_listStyle(style)
        , m_separator(separator)
    {
    }

    const AtomicString& identifier() const { return m_identifier; }
    EListStyleType listStyle() const { return m_listStyle; }
    const AtomicString& separator() const { return m_separator; }

private:
    AtomicString m_identifier;
    EListStyleType m_listStyle;
    AtomicString m_separator;
};

static inline bool operator==(const CounterContent& a, const CounterContent& b)
{
    return a.identifier() == b.identifier()
        && a.listStyle() == b.listStyle()
        && a.separator() == b.separator();
}


} // namespace WebCore

#endif // CounterContent_h
