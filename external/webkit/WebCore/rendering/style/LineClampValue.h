

#ifndef LineClampValue_h
#define LineClampValue_h

#include "RenderStyleConstants.h"

namespace WebCore {
    
class LineClampValue {
public:
    LineClampValue()
        : m_type(LineClampLineCount)
        , m_value(-1)
    {
    }
    
    LineClampValue(int value, ELineClampType type)
        : m_type(type)
        , m_value(value)
    {
    }
    
    int value() const { return m_value; }
    
    bool isPercentage() const { return m_type == LineClampPercentage; }

    bool isNone() const { return m_value == -1; }

    bool operator==(const LineClampValue& o) const
    {
        return value() == o.value() && isPercentage() == o.isPercentage();
    }
    
    bool operator!=(const LineClampValue& o) const
    {
        return !(*this == o);
    }
    
private:
    ELineClampType m_type;
    int m_value;
};
    
} // namespace WebCore

#endif // LineClampValue_h
