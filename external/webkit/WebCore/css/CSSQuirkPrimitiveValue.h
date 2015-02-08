

#ifndef CSSQuirkPrimitiveValue_h
#define CSSQuirkPrimitiveValue_h

#include "CSSPrimitiveValue.h"

namespace WebCore {

// This value is used to handle quirky margins in reflow roots (body, td, and th) like WinIE.
// The basic idea is that a stylesheet can use the value __qem (for quirky em) instead of em.
// When the quirky value is used, if you're in quirks mode, the margin will collapse away
// inside a table cell.
class CSSQuirkPrimitiveValue : public CSSPrimitiveValue {
public:
    static PassRefPtr<CSSQuirkPrimitiveValue> create(double value, UnitTypes type)
    {
        return adoptRef(new CSSQuirkPrimitiveValue(value, type));
    }

private:
    CSSQuirkPrimitiveValue(double num, UnitTypes type)
        : CSSPrimitiveValue(num, type)
    {
    }

    virtual bool isQuirkValue() { return true; }
};

} // namespace WebCore

#endif // CSSQuirkPrimitiveValue_h
