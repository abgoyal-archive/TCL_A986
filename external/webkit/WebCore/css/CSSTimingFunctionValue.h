

#ifndef CSSTimingFunctionValue_h
#define CSSTimingFunctionValue_h

#include "CSSValue.h"
#include <wtf/PassRefPtr.h>

namespace WebCore {

class CSSTimingFunctionValue : public CSSValue {
public:
    static PassRefPtr<CSSTimingFunctionValue> create(double x1, double y1, double x2, double y2)
    {
        return adoptRef(new CSSTimingFunctionValue(x1, y1, x2, y2));
    }

    virtual String cssText() const;

    double x1() const { return m_x1; }
    double y1() const { return m_y1; }
    double x2() const { return m_x2; }
    double y2() const { return m_y2; }

private:
    CSSTimingFunctionValue(double x1, double y1, double x2, double y2)
        : m_x1(x1)
        , m_y1(y1)
        , m_x2(x2)
        , m_y2(y2)
    {
    }

    virtual bool isTimingFunctionValue() const { return true; }
    
    double m_x1;
    double m_y1;
    double m_x2;
    double m_y2;
};

} // namespace

#endif
