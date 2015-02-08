

#include "config.h"
#include "Widget.h"

#include "Assertions.h"
#include "ChromiumBridge.h"

namespace WebCore {

Widget::Widget(PlatformWidget widget)
{
    init(widget);
}

Widget::~Widget() 
{
    ASSERT(!parent());
}

void Widget::show()
{
}

void Widget::hide()
{
}

void Widget::setCursor(const Cursor& cursor)
{
    ChromiumBridge::widgetSetCursor(this, cursor);
}

void Widget::paint(GraphicsContext*, const IntRect&)
{
}

void Widget::setFocus()
{
    ChromiumBridge::widgetSetFocus(this);
}

void Widget::setIsSelected(bool)
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

} // namespace WebCore
