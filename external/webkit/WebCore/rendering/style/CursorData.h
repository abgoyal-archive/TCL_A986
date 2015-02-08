

#ifndef CursorData_h
#define CursorData_h

#include "CachedImage.h"
#include "CachedResourceHandle.h"
#include "IntPoint.h"

namespace WebCore {

struct CursorData {
    CursorData()
        : cursorImage(0)
    {
    }

    bool operator==(const CursorData& o) const
    {
        return hotSpot == o.hotSpot && cursorImage == o.cursorImage;
    }

    bool operator!=(const CursorData& o) const
    {
        return !(*this == o);
    }

    IntPoint hotSpot; // for CSS3 support
    CachedResourceHandle<CachedImage> cursorImage;
};

} // namespace WebCore

#endif // CursorData_h
