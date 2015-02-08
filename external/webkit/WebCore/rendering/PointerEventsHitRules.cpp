

#include "config.h"
#include "PointerEventsHitRules.h"

namespace WebCore {

PointerEventsHitRules::PointerEventsHitRules(EHitTesting hitTesting, EPointerEvents pointerEvents)
    : requireVisible(false)
    , requireFill(false)
    , requireStroke(false)
    , canHitStroke(false)
    , canHitFill(false)
{
    if (hitTesting == SVG_PATH_HITTESTING) {
        switch (pointerEvents)
        {
            case PE_VISIBLE_PAINTED:
            case PE_AUTO: // "auto" is like "visiblePainted" when in SVG content
                requireFill = true;
                requireStroke = true;
            case PE_VISIBLE:
                requireVisible = true;
                canHitFill = true;
                canHitStroke = true;
                break;
            case PE_VISIBLE_FILL:
                requireVisible = true;
                canHitFill = true;
                break;
            case PE_VISIBLE_STROKE:
                requireVisible = true;
                canHitStroke = true;
                break;
            case PE_PAINTED:
                requireFill = true;
                requireStroke = true;
            case PE_ALL:
                canHitFill = true;
                canHitStroke = true;
                break;
            case PE_FILL:
                canHitFill = true;
                break;
            case PE_STROKE:
                canHitStroke = true;
                break;
            case PE_NONE:
                // nothing to do here, defaults are all false.
                break;
        }
    } else {
        switch (pointerEvents)
        {
            case PE_VISIBLE_PAINTED:
            case PE_AUTO: // "auto" is like "visiblePainted" when in SVG content
                requireVisible = true;
                requireFill = true;
                requireStroke = true;
                canHitFill = true;
                canHitStroke = true;
                break;
            case PE_VISIBLE_FILL:
            case PE_VISIBLE_STROKE:
            case PE_VISIBLE:
                requireVisible = true;
                canHitFill = true;
                canHitStroke = true;
                break;
            case PE_PAINTED:
                requireFill = true;
                requireStroke = true;
                canHitFill = true;
                canHitStroke = true;
                break;
            case PE_FILL:
            case PE_STROKE:
            case PE_ALL:
                canHitFill = true;
                canHitStroke = true;
                break;
            case PE_NONE:
                // nothing to do here, defaults are all false.
                break;
        }
    }
}

}

// vim:ts=4:noet
