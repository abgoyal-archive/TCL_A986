

#include "config.h"

#include "Screen.h"
#include "IntRect.h"
#include "FloatRect.h"
#include "Widget.h"

#include <wx/defs.h>
#include <wx/display.h>
#include <wx/gdicmn.h>

namespace WebCore {

FloatRect scaleScreenRectToWidget(FloatRect rect, Widget*)
{
    return rect;
}

FloatRect scaleWidgetRectToScreen(FloatRect rect, Widget*)
{
    return rect;
}

FloatRect screenRect(Widget* widget)
{
    return FloatRect();
}

int screenDepth(Widget* widget)
{
    return wxDisplayDepth();
}

int screenDepthPerComponent(Widget*)
{
    return wxDisplayDepth();
}

bool screenIsMonochrome(Widget* widget)
{
    return wxColourDisplay();
}

FloatRect screenAvailableRect(Widget* widget)
{
    return FloatRect();
}

float scaleFactor(Widget*)
{
    return 1.0f;

}

}
