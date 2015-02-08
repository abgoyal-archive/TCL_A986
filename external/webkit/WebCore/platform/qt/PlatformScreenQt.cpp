

#include "config.h"
#include "PlatformScreen.h"

#include "FloatRect.h"
#include "Frame.h"
#include "FrameView.h"
#include "HostWindow.h"
#include "Widget.h"
#include "QWebPageClient.h"
#include <QApplication>
#include <QDesktopWidget>

namespace WebCore {

static int screenNumber(Widget* w)
{
    if (!w)
        return 0;

    QWebPageClient* client = w->root()->hostWindow()->platformPageClient();
    return client ? client->screenNumber() : 0;
}

int screenDepth(Widget* w)
{
    return QApplication::desktop()->screen(screenNumber(w))->depth();
}

int screenDepthPerComponent(Widget* w)
{
    if (w) {
        QWebPageClient* client = w->root()->hostWindow()->platformPageClient();

        if (client) {
            QWidget* view = client->ownerWidget();
            if (view)
                return view->depth();
        }
    }
    return QApplication::desktop()->screen(0)->depth();
}

bool screenIsMonochrome(Widget* w)
{
    return QApplication::desktop()->screen(screenNumber(w))->numColors() < 2;
}

FloatRect screenRect(Widget* w)
{
    QRect r = QApplication::desktop()->screenGeometry(screenNumber(w));
    return FloatRect(r.x(), r.y(), r.width(), r.height());
}

FloatRect screenAvailableRect(Widget* w)
{
    QRect r = QApplication::desktop()->availableGeometry(screenNumber(w));
    return FloatRect(r.x(), r.y(), r.width(), r.height());
}

} // namespace WebCore
