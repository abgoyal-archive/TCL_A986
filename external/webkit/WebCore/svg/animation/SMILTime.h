

#ifndef SMILTime_h
#define SMILTime_h

#if ENABLE(SVG)

#include <algorithm>

namespace WebCore {

    class SMILTime {
    public:
        SMILTime() : m_time(0) { }
        SMILTime(double time) : m_time(time) { }
        SMILTime(const SMILTime& o) : m_time(o.m_time) { }
        
        static SMILTime unresolved() { return unresolvedValue; }
        static SMILTime indefinite() { return indefiniteValue; }
        
        SMILTime& operator=(const SMILTime& o) { m_time = o.m_time; return *this; }
        double value() const { return m_time; }
        
        bool isFinite() const { return m_time < indefiniteValue; }
        bool isIndefinite() const { return m_time == indefiniteValue; }
        bool isUnresolved() const { return m_time == unresolvedValue; }
        
    private:
        static const double unresolvedValue;
        static const double indefiniteValue;

        double m_time;
    };

    inline bool operator==(const SMILTime& a, const SMILTime& b) { return a.isFinite() && a.value() == b.value(); }
    inline bool operator!=(const SMILTime& a, const SMILTime& b) { return !operator==(a, b); }
    inline bool operator>(const SMILTime& a, const SMILTime& b) { return a.value() > b.value(); }
    inline bool operator<(const SMILTime& a, const SMILTime& b) { return a.value() < b.value(); }
    inline bool operator>=(const SMILTime& a, const SMILTime& b) { return a.value() > b.value() || operator==(a, b); }
    inline bool operator<=(const SMILTime& a, const SMILTime& b) { return a.value() < b.value() || operator==(a, b); }

    SMILTime operator+(const SMILTime&, const SMILTime&);
    SMILTime operator-(const SMILTime&, const SMILTime&);
    // So multiplying times does not make too much sense but SMIL defines it for duration * repeatCount
    SMILTime operator*(const SMILTime&, const SMILTime&);
}

#endif
#endif
