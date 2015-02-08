

#ifndef FontValue_h
#define FontValue_h

#include "CSSValue.h"
#include <wtf/PassRefPtr.h>
#include <wtf/RefPtr.h>

namespace WebCore {

class CSSPrimitiveValue;
class CSSValueList;

class FontValue : public CSSValue {
public:
    static PassRefPtr<FontValue> create()
    {
        return adoptRef(new FontValue);
    }

    virtual String cssText() const;

    RefPtr<CSSPrimitiveValue> style;
    RefPtr<CSSPrimitiveValue> variant;
    RefPtr<CSSPrimitiveValue> weight;
    RefPtr<CSSPrimitiveValue> size;
    RefPtr<CSSPrimitiveValue> lineHeight;
    RefPtr<CSSValueList> family;

private:
    FontValue() { }

    virtual bool isFontValue() const { return true; }
};

} // namespace

#endif
