

#ifndef PlatformScrollbar_h
#define PlatformScrollbar_h

#include "Scrollbar.h"
#include "Timer.h"
#include <wtf/PassRefPtr.h>

typedef struct HDC__* HDC;

namespace WebCore {

class PlatformScrollbar : public Scrollbar {
public:
    static PassRefPtr<PlatformScrollbar> create(ScrollbarClient* client, ScrollbarOrientation orientation, ScrollbarControlSize size)
    {
        return adoptRef(new PlatformScrollbar(client, orientation, size));
    }
    
    PlatformScrollbar(ScrollbarClient*, ScrollbarOrientation, ScrollbarControlSize, ScrollbarTheme* = 0);
};

}

#endif // PlatformScrollbar_h

