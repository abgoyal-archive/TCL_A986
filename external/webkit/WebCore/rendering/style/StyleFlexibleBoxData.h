

#ifndef StyleFlexibleBoxData_h
#define StyleFlexibleBoxData_h

#include <wtf/RefCounted.h>
#include <wtf/PassRefPtr.h>

namespace WebCore {

class StyleFlexibleBoxData : public RefCounted<StyleFlexibleBoxData> {
public:
    static PassRefPtr<StyleFlexibleBoxData> create() { return adoptRef(new StyleFlexibleBoxData); }
    PassRefPtr<StyleFlexibleBoxData> copy() const { return adoptRef(new StyleFlexibleBoxData(*this)); }
    
    bool operator==(const StyleFlexibleBoxData& o) const;
    bool operator!=(const StyleFlexibleBoxData& o) const
    {
        return !(*this == o);
    }

    float flex;
    unsigned int flex_group;
    unsigned int ordinal_group;

    unsigned align : 3; // EBoxAlignment
    unsigned pack: 3; // EBoxAlignment
    unsigned orient: 1; // EBoxOrient
    unsigned lines : 1; // EBoxLines
    
private:
    StyleFlexibleBoxData();
    StyleFlexibleBoxData(const StyleFlexibleBoxData&);
};

} // namespace WebCore

#endif // StyleFlexibleBoxData_h
