
#include "config.h"
#include "Widget.h"

#include "Cursor.h"
#include "GraphicsContext.h"
#include "IntRect.h"
#include "NotImplemented.h"

#include <wx/defs.h>
#include <wx/scrolwin.h>

namespace WebCore {

Widget::Widget(PlatformWidget widget)
{
    init(widget);
}

Widget::~Widget()
{
}

void Widget::setFocus()
{
    if (PlatformWidget widget = platformWidget())
        widget->SetFocus();
}

void Widget::setCursor(const Cursor& cursor)
{
    if (platformWidget() && cursor.impl())
        platformWidget()->SetCursor(*cursor.impl());
}

void Widget::show()
{
    if (PlatformWidget widget = platformWidget())
        widget->Show();
}

void Widget::hide()
{
    if (PlatformWidget widget = platformWidget())
        widget->Hide();
}

IntRect Widget::frameRect() const
{
    if (PlatformWidget widget = platformWidget())
        return widget->GetRect();
    
    return m_frame;
}

void Widget::setFrameRect(const IntRect& rect)
{
    if (PlatformWidget widget = platformWidget())
        widget->SetSize(rect);
    
    m_frame = rect;
}

void Widget::invalidateRect(const IntRect& r)
{
    if (PlatformWidget widget = platformWidget())
        widget->RefreshRect(r);
}

void Widget::paint(GraphicsContext*,const IntRect& r)
{
    invalidateRect(r);
    if (PlatformWidget widget = platformWidget())
        widget->Update();
}

void Widget::setIsSelected(bool)
{
    notImplemented();
}

}
