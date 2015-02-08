

#ifndef StyleBackgroundData_h
#define StyleBackgroundData_h

#include "Color.h"
#include "FillLayer.h"
#include "OutlineValue.h"
#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>

namespace WebCore {

class StyleBackgroundData : public RefCounted<StyleBackgroundData> {
public:
    static PassRefPtr<StyleBackgroundData> create() { return adoptRef(new StyleBackgroundData); }
    PassRefPtr<StyleBackgroundData> copy() const { return adoptRef(new StyleBackgroundData(*this)); }
    ~StyleBackgroundData() { }

    bool operator==(const StyleBackgroundData& o) const;
    bool operator!=(const StyleBackgroundData& o) const
    {
        return !(*this == o);
    }

    FillLayer m_background;
    Color m_color;
    OutlineValue m_outline;

private:
    StyleBackgroundData();
    StyleBackgroundData(const StyleBackgroundData&);    
};

} // namespace WebCore

#endif // StyleBackgroundData_h
