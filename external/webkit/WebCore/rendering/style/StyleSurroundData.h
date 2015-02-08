

#ifndef StyleSurroundData_h
#define StyleSurroundData_h

#include "BorderData.h"
#include "LengthBox.h"
#include <wtf/RefCounted.h>
#include <wtf/PassRefPtr.h>

namespace WebCore {

class StyleSurroundData : public RefCounted<StyleSurroundData> {
public:
    static PassRefPtr<StyleSurroundData> create() { return adoptRef(new StyleSurroundData); }
    PassRefPtr<StyleSurroundData> copy() const { return adoptRef(new StyleSurroundData(*this)); }
    
    bool operator==(const StyleSurroundData& o) const;
    bool operator!=(const StyleSurroundData& o) const
    {
        return !(*this == o);
    }

    LengthBox offset;
    LengthBox margin;
    LengthBox padding;
    BorderData border;
    
private:
    StyleSurroundData();
    StyleSurroundData(const StyleSurroundData&);    
};

} // namespace WebCore

#endif // StyleSurroundData_h
