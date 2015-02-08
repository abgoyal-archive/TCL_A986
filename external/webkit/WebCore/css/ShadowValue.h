

#ifndef ShadowValue_h
#define ShadowValue_h

#include "CSSValue.h"
#include <wtf/PassRefPtr.h>
#include <wtf/RefPtr.h>

namespace WebCore {

class CSSPrimitiveValue;

// Used for text-shadow and box-shadow
class ShadowValue : public CSSValue {
public:
    static PassRefPtr<ShadowValue> create(PassRefPtr<CSSPrimitiveValue> x,
        PassRefPtr<CSSPrimitiveValue> y,
        PassRefPtr<CSSPrimitiveValue> blur,
        PassRefPtr<CSSPrimitiveValue> spread,
        PassRefPtr<CSSPrimitiveValue> style,
        PassRefPtr<CSSPrimitiveValue> color)
    {
        return adoptRef(new ShadowValue(x, y, blur, spread, style, color));
    }

    virtual String cssText() const;

    RefPtr<CSSPrimitiveValue> x;
    RefPtr<CSSPrimitiveValue> y;
    RefPtr<CSSPrimitiveValue> blur;
    RefPtr<CSSPrimitiveValue> spread;
    RefPtr<CSSPrimitiveValue> style;
    RefPtr<CSSPrimitiveValue> color;

private:
    ShadowValue(PassRefPtr<CSSPrimitiveValue> x,
        PassRefPtr<CSSPrimitiveValue> y,
        PassRefPtr<CSSPrimitiveValue> blur,
        PassRefPtr<CSSPrimitiveValue> spread,
        PassRefPtr<CSSPrimitiveValue> style,
        PassRefPtr<CSSPrimitiveValue> color);
};

} // namespace

#endif
