

#ifndef Pair_h
#define Pair_h

#include <wtf/RefCounted.h>
#include "CSSPrimitiveValue.h"
#include <wtf/PassRefPtr.h>

namespace WebCore {

// A primitive value representing a pair.  This is useful for properties like border-radius, background-size/position,
// and border-spacing (all of which are space-separated sets of two values).  At the moment we are only using it for
// border-radius and background-size, but (FIXME) border-spacing and background-position could be converted over to use
// it (eliminating some extra -webkit- internal properties).
class Pair : public RefCounted<Pair> {
public:
    static PassRefPtr<Pair> create()
    {
        return adoptRef(new Pair);
    }
    static PassRefPtr<Pair> create(PassRefPtr<CSSPrimitiveValue> first, PassRefPtr<CSSPrimitiveValue> second)
    {
        return adoptRef(new Pair(first, second));
    }
    virtual ~Pair() { }

    CSSPrimitiveValue* first() const { return m_first.get(); }
    CSSPrimitiveValue* second() const { return m_second.get(); }

    void setFirst(PassRefPtr<CSSPrimitiveValue> first) { m_first = first; }
    void setSecond(PassRefPtr<CSSPrimitiveValue> second) { m_second = second; }

private:
    Pair() : m_first(0), m_second(0) { }
    Pair(PassRefPtr<CSSPrimitiveValue> first, PassRefPtr<CSSPrimitiveValue> second)
        : m_first(first), m_second(second) { }
    
    RefPtr<CSSPrimitiveValue> m_first;
    RefPtr<CSSPrimitiveValue> m_second;
};

} // namespace

#endif
