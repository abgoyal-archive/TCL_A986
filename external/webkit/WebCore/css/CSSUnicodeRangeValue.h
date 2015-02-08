

#ifndef CSSUnicodeRangeValue_h
#define CSSUnicodeRangeValue_h

#include "CSSValue.h"
#include <wtf/PassRefPtr.h>
#include <wtf/unicode/Unicode.h>

namespace WebCore {

class CSSUnicodeRangeValue : public CSSValue {
public:
    static PassRefPtr<CSSUnicodeRangeValue> create(UChar32 from, UChar32 to)
    {
        return adoptRef(new CSSUnicodeRangeValue(from, to));
    }

    virtual ~CSSUnicodeRangeValue();

    UChar32 from() const { return m_from; }
    UChar32 to() const { return m_to; }

    virtual String cssText() const;

private:
    CSSUnicodeRangeValue(UChar32 from, UChar32 to)
        : m_from(from)
        , m_to(to)
    {
    }

    UChar32 m_from;
    UChar32 m_to;
};

} // namespace WebCore

#endif // CSSUnicodeRangeValue_h
