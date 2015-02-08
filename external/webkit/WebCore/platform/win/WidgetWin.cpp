

#include "config.h"
#include "Widget.h"

#include "Chrome.h"
#include "Cursor.h"
#include "Document.h"
#include "Element.h"
#include "FrameView.h"
#include "FrameWin.h"
#include "GraphicsContext.h"
#include "IntRect.h"
#include "Page.h"

#include <winsock2.h>
#include <windows.h>

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

HCURSOR lastSetCursor = 0;
bool ignoreNextSetCursor = false;

void Widget::setCursor(const Cursor& cursor)
{
    // This is set by PluginViewWin so it can ignore the setCursor call made by
    // EventHandler.cpp.
    if (ignoreNextSetCursor) {
        ignoreNextSetCursor = false;
        return;
    }

    if (!cursor.impl()->nativeCursor())
        return;

    lastSetCursor = cursor.impl()->nativeCursor();

    ScrollView* view = root();
    if (!view || !view->isFrameView()) {
        SetCursor(lastSetCursor);
        return;
    }

    Frame* frame = static_cast<FrameView*>(view)->frame();
    if (!frame) {
        SetCursor(lastSetCursor);
        return;
    }

    Page* page = frame->page();
    if (!page) {
        SetCursor(lastSetCursor);
        return;
    }

    page->chrome()->setCursor(lastSetCursor);
}

void Widget::paint(GraphicsContext*, const IntRect&)
{
}

void Widget::setFocus()
{
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
