

#include "config.h"
#include "Screen.h"

#include "Page.h"
#include "Frame.h"
#include "FrameView.h"
#include "Widget.h"
#include "IntRect.h"
#include "FloatRect.h"

#include <QApplication>
#include <QDesktopWidget>

namespace WebCore {

static QWidget* qwidgetForPage(const Page* page)
{
    Frame* frame = (page ? page->mainFrame() : 0);
    FrameView* frameView = (frame ? frame->view() : 0);

    if (!frameView)
        return 0;

    return frameView->qwidget();
}

FloatRect screenRect(const Page* page)
{
    QWidget* qw = qwidgetForPage(page);
    if (!qw)
        return FloatRect();

    // Taken from KGlobalSettings::desktopGeometry
    QDesktopWidget* dw = QApplication::desktop();
    if (!dw)
        return FloatRect();

    return IntRect(dw->screenGeometry(qw));
}

int screenDepth(const Page* page)
{
    QWidget* qw = qwidgetForPage(page);
    if (!qw)
        return 32;

    return qw->depth();
}

FloatRect usableScreenRect(const Page* page)
{
    QWidget* qw = qwidgetForPage(page);
    if (!qw)
        return FloatRect();

    // Taken from KGlobalSettings::desktopGeometry
    QDesktopWidget* dw = QApplication::desktop();
    if (!dw)
        return FloatRect();

    return IntRect(dw->availableGeometry(qw));
}

float scaleFactor(const Page*)
{
    return 1.0f;
}

}

// vim: ts=4 sw=4 et
