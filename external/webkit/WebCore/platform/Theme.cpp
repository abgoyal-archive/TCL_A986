

#include "config.h"
#include "Theme.h"

namespace WebCore {

LengthBox Theme::controlBorder(ControlPart part, const Font&, const LengthBox& zoomedBox, float) const
{
    switch (part) {
        case PushButtonPart:
        case MenulistPart:
        case SearchFieldPart:
        case CheckboxPart:
        case RadioPart:
            return LengthBox(0);
        default:
            return zoomedBox;
    }
}

LengthBox Theme::controlPadding(ControlPart part, const Font&, const LengthBox& zoomedBox, float) const
{
    switch (part) {
        case MenulistPart:
        case MenulistButtonPart:
        case CheckboxPart:
        case RadioPart:
            return LengthBox(0);
        default:
            return zoomedBox;
    }
}

}
