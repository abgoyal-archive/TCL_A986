

#ifndef StyleMarqueeData_h
#define StyleMarqueeData_h

#include "Length.h"
#include "RenderStyleConstants.h"
#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>

namespace WebCore {

class StyleMarqueeData : public RefCounted<StyleMarqueeData> {
public:
    static PassRefPtr<StyleMarqueeData> create() { return adoptRef(new StyleMarqueeData); }
    PassRefPtr<StyleMarqueeData> copy() const { return adoptRef(new StyleMarqueeData(*this)); }

    bool operator==(const StyleMarqueeData& o) const;
    bool operator!=(const StyleMarqueeData& o) const
    {
        return !(*this == o);
    }

    Length increment;
    int speed;

    int loops; // -1 means infinite.

    unsigned behavior : 2; // EMarqueeBehavior 
    EMarqueeDirection direction : 3; // not unsigned because EMarqueeDirection has negative values

private:
    StyleMarqueeData();
    StyleMarqueeData(const StyleMarqueeData&);
};

} // namespace WebCore

#endif // StyleMarqueeData_h
