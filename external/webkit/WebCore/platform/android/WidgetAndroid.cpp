

#include "config.h"
#include "Widget.h"

#include "Font.h"
#include "FrameView.h"
#include "GraphicsContext.h"
#include "NotImplemented.h"
#include "WebCoreFrameBridge.h"
#include "WebCoreViewBridge.h"
#include "WebViewCore.h"

namespace WebCore {

Widget::Widget(PlatformWidget widget)
{
    init(widget);
}

Widget::~Widget()
{
    ASSERT(!parent());
    releasePlatformWidget();
}

IntRect Widget::frameRect() const
{
    if (!platformWidget())
        return m_frame;
    return platformWidget()->getBounds();
}

void Widget::setFocus()
{
    notImplemented();
}

void Widget::paint(GraphicsContext* ctx, const IntRect& r)
{
    // FIXME: in what case, will this be called for the top frame?
    if (!platformWidget())
        return;
    platformWidget()->draw(ctx, r);
}

void Widget::releasePlatformWidget()
{
    Release(platformWidget());
}

void Widget::retainPlatformWidget()
{
    Retain(platformWidget());
}

void Widget::setCursor(const Cursor& cursor)
{
    notImplemented();
}

void Widget::show()
{
    notImplemented(); 
}

void Widget::hide()
{
    notImplemented(); 
}

void Widget::setFrameRect(const IntRect& rect)
{
    m_frame = rect;
    // platformWidget() is 0 when called from Scrollbar
    if (!platformWidget())
        return;
    platformWidget()->setLocation(rect.x(), rect.y());
    platformWidget()->setSize(rect.width(), rect.height());
}

void Widget::setIsSelected(bool isSelected)
{
    notImplemented();
}

int Widget::screenWidth() const
{
    const Widget* widget = this;
    while (!widget->isFrameView()) {
        widget = widget->parent();
        if (!widget)
            break;
    }
    if (!widget)
        return 0;
    android::WebViewCore* core = android::WebViewCore::getWebViewCore(
        static_cast<const ScrollView*>(widget));
    if (!core)
        return 0;
    return core->screenWidth();
}

} // WebCore namepsace
