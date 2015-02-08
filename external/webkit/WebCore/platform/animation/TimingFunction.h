

#ifndef TimingFunction_h
#define TimingFunction_h

#include "RenderStyleConstants.h"

namespace WebCore {

struct TimingFunction {
    TimingFunction()
        : m_type(CubicBezierTimingFunction)
        , m_x1(0.25)
        , m_y1(0.1)
        , m_x2(0.25)
        , m_y2(1.0)
    {
    }

    TimingFunction(ETimingFunctionType timingFunction, double x1 = 0.0, double y1 = 0.0, double x2 = 1.0, double y2 = 1.0)
        : m_type(timingFunction)
        , m_x1(x1)
        , m_y1(y1)
        , m_x2(x2)
        , m_y2(y2)
    {
    }

    bool operator==(const TimingFunction& o) const
    {
        return m_type == o.m_type && m_x1 == o.m_x1 && m_y1 == o.m_y1 && m_x2 == o.m_x2 && m_y2 == o.m_y2;
    }

    double x1() const { return m_x1; }
    double y1() const { return m_y1; }
    double x2() const { return m_x2; }
    double y2() const { return m_y2; }

    ETimingFunctionType type() const { return m_type; }

private:
    ETimingFunctionType m_type;

    double m_x1;
    double m_y1;
    double m_x2;
    double m_y2;
};

} // namespace WebCore

#endif // TimingFunction_h
