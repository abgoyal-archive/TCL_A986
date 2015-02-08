

#ifndef FontCustomPlatformData_h_
#define FontCustomPlatformData_h_

#include "FontRenderingMode.h"
#include <wtf/Noncopyable.h>

class SkTypeface;

namespace WebCore {
    
    class SharedBuffer;
    class FontPlatformData;
    
    class FontCustomPlatformData : public Noncopyable {
    public:
        FontCustomPlatformData(SkTypeface* face);
        ~FontCustomPlatformData();

        SkTypeface* typeface() const { return m_typeface; }
        
        FontPlatformData fontPlatformData(int size, bool bold, bool italic, FontRenderingMode );
        
    private:
        SkTypeface* m_typeface;
    };
    
    FontCustomPlatformData* createFontCustomPlatformData(SharedBuffer* buffer);
    
} // namespace WebCore

#endif // FontCustomPlatformData_h_
