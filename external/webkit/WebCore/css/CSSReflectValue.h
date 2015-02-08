

#ifndef CSSReflectValue_h
#define CSSReflectValue_h

#include "CSSReflectionDirection.h"
#include "CSSValue.h"
#include <wtf/PassRefPtr.h>
#include <wtf/RefPtr.h>

namespace WebCore {

class CSSPrimitiveValue;

class CSSReflectValue : public CSSValue {
public:
    static PassRefPtr<CSSReflectValue> create(CSSReflectionDirection direction,
        PassRefPtr<CSSPrimitiveValue> offset, PassRefPtr<CSSValue> mask)
    {
        return adoptRef(new CSSReflectValue(direction, offset, mask));
    }

    CSSReflectionDirection direction() const { return m_direction; }
    CSSPrimitiveValue* offset() const { return m_offset.get(); }
    CSSValue* mask() const { return m_mask.get(); }

    virtual String cssText() const;

    virtual void addSubresourceStyleURLs(ListHashSet<KURL>&, const CSSStyleSheet*);

private:
    CSSReflectValue(CSSReflectionDirection direction,
            PassRefPtr<CSSPrimitiveValue> offset, PassRefPtr<CSSValue> mask)
        : m_direction(direction)
        , m_offset(offset)
        , m_mask(mask)
    {
    }
    
    CSSReflectionDirection m_direction;
    RefPtr<CSSPrimitiveValue> m_offset;
    RefPtr<CSSValue> m_mask;
};

} // namespace WebCore

#endif // CSSReflectValue_h
