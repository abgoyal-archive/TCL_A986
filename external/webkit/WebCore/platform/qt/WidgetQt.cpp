

#include "config.h"
#include "Widget.h"

#include "Cursor.h"
#include "Font.h"
#include "GraphicsContext.h"
#include "HostWindow.h"
#include "IntRect.h"
#include "ScrollView.h"
#include "NotImplemented.h"
#include "QWebPageClient.h"

#include "qwebframe.h"
#include "qwebframe_p.h"
#include "qwebpage.h"

#include <QCoreApplication>
#include <QPainter>
#include <QPaintEngine>

#include <QDebug>

namespace WebCore {

Widget::Widget(QWidget* widget)
{
    init(widget);
}

Widget::~Widget()
{
    Q_ASSERT(!parent());
}

IntRect Widget::frameRect() const
{
    return m_frame;
}

void Widget::setFrameRect(const IntRect& rect)
{
    m_frame = rect;

    frameRectsChanged();
}

void Widget::setFocus()
{
}

void Widget::setCursor(const Cursor& cursor)
{
#ifndef QT_NO_CURSOR
    QWebPageClient* pageClient = root()->hostWindow()->platformPageClient();

    if (pageClient)
        pageClient->setCursor(cursor.impl());
#endif
}

void Widget::show()
{
    setSelfVisible(true);

    if (isParentVisible() && platformWidget())
        platformWidget()->show();
}

void Widget::hide()
{
    setSelfVisible(false);

    if (isParentVisible() && platformWidget())
        platformWidget()->hide();
}

void Widget::paint(GraphicsContext*, const IntRect&)
{
}

void Widget::setIsSelected(bool)
{
    notImplemented();
}

}

// vim: ts=4 sw=4 et
