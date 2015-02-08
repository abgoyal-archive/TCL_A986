

#ifndef CSSInitialValue_h
#define CSSInitialValue_h

#include "CSSValue.h"
#include <wtf/PassRefPtr.h>

namespace WebCore {

class CSSInitialValue : public CSSValue {
public:
    static PassRefPtr<CSSInitialValue> createExplicit()
    {
        static CSSInitialValue* explicitValue = new CSSInitialValue(false);
        return explicitValue;
    }
    static PassRefPtr<CSSInitialValue> createImplicit()
    {
        static CSSInitialValue* explicitValue = new CSSInitialValue(true);
        return explicitValue;
    }

    virtual String cssText() const;
        
private:
    CSSInitialValue(bool implicit)
        : m_implicit(implicit)
    {
    }

    virtual unsigned short cssValueType() const;
    virtual bool isImplicitInitialValue() const { return m_implicit; }

    bool m_implicit;
};

} // namespace WebCore

#endif // CSSInitialValue_h
