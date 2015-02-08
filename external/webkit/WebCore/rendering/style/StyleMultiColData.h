

#ifndef StyleMultiColData_h
#define StyleMultiColData_h

#include "BorderValue.h"
#include "Length.h"
#include "RenderStyleConstants.h"
#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>

namespace WebCore {

// CSS3 Multi Column Layout

class StyleMultiColData : public RefCounted<StyleMultiColData> {
public:
    static PassRefPtr<StyleMultiColData> create() { return adoptRef(new StyleMultiColData); }
    PassRefPtr<StyleMultiColData> copy() const { return adoptRef(new StyleMultiColData(*this)); }
    
    bool operator==(const StyleMultiColData& o) const;
    bool operator!=(const StyleMultiColData &o) const
    {
        return !(*this == o);
    }

    unsigned short ruleWidth() const
    {
        if (m_rule.style() == BNONE || m_rule.style() == BHIDDEN)
            return 0; 
        return m_rule.width;
    }

    float m_width;
    unsigned short m_count;
    float m_gap;
    BorderValue m_rule;
    
    bool m_autoWidth : 1;
    bool m_autoCount : 1;
    bool m_normalGap : 1;
    unsigned m_breakBefore : 2; // EPageBreak
    unsigned m_breakAfter : 2; // EPageBreak
    unsigned m_breakInside : 2; // EPageBreak
    
private:
    StyleMultiColData();
    StyleMultiColData(const StyleMultiColData&);
};

} // namespace WebCore

#endif // StyleMultiColData_h
