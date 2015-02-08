

#include "config.h"
#include "Widget.h"

#include "Cursor.h"
#include "GraphicsContext.h"
#include "IntRect.h"
#include "NotImplemented.h"
#include <Control.h>
#include <View.h>


namespace WebCore {

Widget::Widget(PlatformWidget widget)
{
    init(widget);
}

Widget::~Widget()
{
}

IntRect Widget::frameRect() const
{
    return m_frame;
}

void Widget::setFrameRect(const IntRect& rect)
{
    m_frame = rect;
}

void Widget::setFocus()
{
    if (platformWidget())
        platformWidget()->MakeFocus();
}

void Widget::setCursor(const Cursor& cursor)
{
    if (platformWidget())
        platformWidget()->SetViewCursor(cursor.impl());
}

void Widget::show()
{
    if (platformWidget())
        platformWidget()->Show();
}

void Widget::hide()
{
    if (platformWidget())
        platformWidget()->Hide();
}

void Widget::paint(GraphicsContext* p, IntRect const& r)
{
    notImplemented();
}

void Widget::setIsSelected(bool)
{
    notImplemented();
}

} // namespace WebCore

