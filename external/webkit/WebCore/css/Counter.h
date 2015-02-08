

#ifndef Counter_h
#define Counter_h

#include "CSSPrimitiveValue.h"
#include "PlatformString.h"

namespace WebCore {

class Counter : public RefCounted<Counter> {
public:
    static PassRefPtr<Counter> create(PassRefPtr<CSSPrimitiveValue> identifier, PassRefPtr<CSSPrimitiveValue> listStyle, PassRefPtr<CSSPrimitiveValue> separator)
    {
        return adoptRef(new Counter(identifier, listStyle, separator));
    }

    String identifier() const { return m_identifier ? m_identifier->getStringValue() : String(); }
    String listStyle() const { return m_listStyle ? m_listStyle->getStringValue() : String(); }
    String separator() const { return m_separator ? m_separator->getStringValue() : String(); }

    int listStyleNumber() const { return m_listStyle ? m_listStyle->getIntValue() : 0; }

    void setIdentifier(PassRefPtr<CSSPrimitiveValue> identifier) { m_identifier = identifier; }
    void setListStyle(PassRefPtr<CSSPrimitiveValue> listStyle) { m_listStyle = listStyle; }
    void setSeparator(PassRefPtr<CSSPrimitiveValue> separator) { m_separator = separator; }

private:
    Counter(PassRefPtr<CSSPrimitiveValue> identifier, PassRefPtr<CSSPrimitiveValue> listStyle, PassRefPtr<CSSPrimitiveValue> separator)
        : m_identifier(identifier)
        , m_listStyle(listStyle)
        , m_separator(separator)
    {
    }

    RefPtr<CSSPrimitiveValue> m_identifier; // string
    RefPtr<CSSPrimitiveValue> m_listStyle; // int
    RefPtr<CSSPrimitiveValue> m_separator; // string
};

} // namespace WebCore

#endif // Counter_h
