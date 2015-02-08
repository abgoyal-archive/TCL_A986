

#ifndef ColorMac_h
#define ColorMac_h

#include "Color.h"

#ifdef __OBJC__
@class NSColor;
#else
class NSColor;
#endif

namespace WebCore {
    
    // These functions assume NSColors are in DeviceRGB colorspace
    Color colorFromNSColor(NSColor *);
    NSColor *nsColor(const Color&);

    bool usesTestModeFocusRingColor();
    void setUsesTestModeFocusRingColor(bool);
    
    // Focus ring color used for testing purposes.
    RGBA32 oldAquaFocusRingColor();

}

#endif
