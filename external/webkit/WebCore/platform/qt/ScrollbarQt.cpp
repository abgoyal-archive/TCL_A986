

#include "config.h"
#include "Scrollbar.h"

#include "EventHandler.h"
#include "FrameView.h"
#include "Frame.h"
#include "GraphicsContext.h"
#include "IntRect.h"
#include "PlatformMouseEvent.h"
#include "ScrollbarTheme.h"

#include <QApplication>
#include <QDebug>
#include <QPainter>
#include <QStyle>
#include <QMenu>

using namespace std;

namespace WebCore {

bool Scrollbar::contextMenu(const PlatformMouseEvent& event)
{
#ifndef QT_NO_CONTEXTMENU
    if (!QApplication::style()->styleHint(QStyle::SH_ScrollBar_ContextMenu))
        return true;

    bool horizontal = (m_orientation == HorizontalScrollbar);

    QMenu menu;
    QAction* actScrollHere = menu.addAction(QCoreApplication::translate("QWebPage", "Scroll here"));
    menu.addSeparator();

    QAction* actScrollTop = menu.addAction(horizontal ? QCoreApplication::translate("QWebPage", "Left edge") : QCoreApplication::translate("QWebPage", "Top"));
    QAction* actScrollBottom = menu.addAction(horizontal ? QCoreApplication::translate("QWebPage", "Right edge") : QCoreApplication::translate("QWebPage", "Bottom"));
    menu.addSeparator();

    QAction* actPageUp = menu.addAction(horizontal ? QCoreApplication::translate("QWebPage", "Page left") : QCoreApplication::translate("QWebPage", "Page up"));
    QAction* actPageDown = menu.addAction(horizontal ? QCoreApplication::translate("QWebPage", "Page right") : QCoreApplication::translate("QWebPage", "Page down"));
    menu.addSeparator();

    QAction* actScrollUp = menu.addAction(horizontal ? QCoreApplication::translate("QWebPage", "Scroll left") : QCoreApplication::translate("QWebPage", "Scroll up"));
    QAction* actScrollDown = menu.addAction(horizontal ? QCoreApplication::translate("QWebPage", "Scroll right") : QCoreApplication::translate("QWebPage", "Scroll down"));

    const QPoint globalPos = QPoint(event.globalX(), event.globalY());
    QAction* actionSelected = menu.exec(globalPos);

    if (!actionSelected)
        { /* Do nothing */ }
    else if (actionSelected == actScrollHere) {
        const QPoint pos = convertFromContainingWindow(event.pos());
        moveThumb(horizontal ? pos.x() : pos.y());
    } else if (actionSelected == actScrollTop)
        setValue(0);
    else if (actionSelected == actScrollBottom)
        setValue(maximum());
    else if (actionSelected == actPageUp)
        scroll(horizontal ? ScrollLeft: ScrollUp, ScrollByPage, 1);
    else if (actionSelected == actPageDown)
        scroll(horizontal ? ScrollRight : ScrollDown, ScrollByPage, 1);
    else if (actionSelected == actScrollUp)
        scroll(horizontal ? ScrollLeft : ScrollUp, ScrollByLine, 1);
    else if (actionSelected == actScrollDown)
        scroll(horizontal ? ScrollRight : ScrollDown, ScrollByLine, 1);
#endif // QT_NO_CONTEXTMENU
    return true;
}

}

// vim: ts=4 sw=4 et
