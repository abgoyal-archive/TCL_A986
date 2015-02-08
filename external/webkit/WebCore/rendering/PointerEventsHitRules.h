

#ifndef PointerEventsHitRules_h
#define PointerEventsHitRules_h

#include "RenderStyleConstants.h"

namespace WebCore {

class PointerEventsHitRules {
public:
    enum EHitTesting {
        SVG_IMAGE_HITTESTING,
        SVG_PATH_HITTESTING,
        SVG_TEXT_HITTESTING
    };

    PointerEventsHitRules(EHitTesting, EPointerEvents);

    bool requireVisible;
    bool requireFill;
    bool requireStroke;
    bool canHitStroke;
    bool canHitFill;  
};

}

#endif

// vim:ts=4:noet
