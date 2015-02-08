

#ifndef TimeRanges_h
#define TimeRanges_h

#include "ExceptionCode.h"

#include <algorithm>
#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>
#include <wtf/Vector.h>

namespace WebCore {

class TimeRanges : public RefCounted<TimeRanges> {
public:
    static PassRefPtr<TimeRanges> create() 
    {
        return adoptRef(new TimeRanges);
    }
    static PassRefPtr<TimeRanges> create(float start, float end)
    {
        return adoptRef(new TimeRanges(start, end));
    }

    PassRefPtr<TimeRanges> copy();

    unsigned length() const { return m_ranges.size(); }
    float start(unsigned index, ExceptionCode&) const;
    float end(unsigned index, ExceptionCode&) const;
    
    void add(float start, float end);
    
    bool contain(float time) const;

private:
    TimeRanges() { }
    TimeRanges(float start, float end);
    TimeRanges(const TimeRanges&);

    // We consider all the Ranges to be semi-bounded as follow: [start, end[
    struct Range {
        Range() { }
        Range(float start, float end)
        {
            m_start = start;
            m_end = end;
        }
        float m_start;
        float m_end;

        inline bool isPointInRange(float point) const
        {
            return m_start <= point && point < m_end;
        }
        
        inline bool isOverlappingRange(const Range& range) const
        {
            return isPointInRange(range.m_start) || isPointInRange(range.m_end) || range.isPointInRange(m_start);
        }

        inline bool isContiguousWithRange(const Range& range) const
        {
            return range.m_start == m_end || range.m_end == m_start;
        }
        
        inline Range unionWithOverlappingOrContiguousRange(const Range& range) const
        {
            Range ret;

            ret.m_start = std::min(m_start, range.m_start);
            ret.m_end = std::max(m_end, range.m_end);

            return ret;
        }

        inline bool isBeforeRange(const Range& range) const
        {
            return range.m_start >= m_end;
        }
    };
    
    Vector<Range> m_ranges;
};

} // namespace WebCore

#endif
