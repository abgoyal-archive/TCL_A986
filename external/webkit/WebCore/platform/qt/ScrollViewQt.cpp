

#include "config.h"
#include "ScrollView.h"

namespace WebCore {

void ScrollView::platformInit()
{
}

void ScrollView::platformDestroy()
{
}

void ScrollView::platformAddChild(Widget*)
{
}

void ScrollView::platformRemoveChild(Widget* child)
{
    child->hide();
}

}
// vim: ts=4 sw=4 et
