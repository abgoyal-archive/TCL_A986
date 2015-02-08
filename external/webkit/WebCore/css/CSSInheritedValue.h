

#ifndef CSSInheritedValue_h
#define CSSInheritedValue_h

#include "CSSValue.h"
#include <wtf/PassRefPtr.h>

namespace WebCore {

class CSSInheritedValue : public CSSValue {
public:
    static PassRefPtr<CSSInheritedValue> create()
    {
        return adoptRef(new CSSInheritedValue);
    }

    virtual String cssText() const;

private:
    CSSInheritedValue() { }
    virtual unsigned short cssValueType() const;
};

} // namespace WebCore

#endif // CSSInheritedValue_h
