

#ifndef StyleBoxData_h
#define StyleBoxData_h

#include "Length.h"
#include <wtf/RefCounted.h>
#include <wtf/PassRefPtr.h>

namespace WebCore {

class StyleBoxData : public RefCounted<StyleBoxData> {
public:
    static PassRefPtr<StyleBoxData> create() { return adoptRef(new StyleBoxData); }
    PassRefPtr<StyleBoxData> copy() const { return adoptRef(new StyleBoxData(*this)); }

    bool operator==(const StyleBoxData& o) const;
    bool operator!=(const StyleBoxData& o) const
    {
        return !(*this == o);
    }

    Length width;
    Length height;

    Length min_width;
    Length max_width;

    Length min_height;
    Length max_height;

    Length vertical_align;

    int z_index;
    bool z_auto : 1;
    unsigned boxSizing : 1; // EBoxSizing
    
private:
    StyleBoxData();
    StyleBoxData(const StyleBoxData&);
};

} // namespace WebCore

#endif // StyleBoxData_h
