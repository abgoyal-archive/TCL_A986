

#include "config.h"
#include "qwebhistoryinterface.h"

#include <QCoreApplication>

#include "PageGroup.h"
#include "PlatformString.h"


static QWebHistoryInterface* default_interface;

static bool gRoutineAdded;

static void gCleanupInterface()
{
    if (default_interface && !default_interface->parent())
        delete default_interface;
    default_interface = 0;
}

void QWebHistoryInterface::setDefaultInterface(QWebHistoryInterface* defaultInterface)
{
    if (default_interface == defaultInterface)
        return;

    if (default_interface && !default_interface->parent())
        delete default_interface;

    default_interface = defaultInterface;
    WebCore::PageGroup::removeAllVisitedLinks();

    //### enable after the introduction of a version
    //WebCore::PageGroup::setShouldTrackVisitedLinks(true);

    if (!gRoutineAdded) {
        qAddPostRoutine(gCleanupInterface);
        gRoutineAdded = true;
    }
}

QWebHistoryInterface* QWebHistoryInterface::defaultInterface()
{
    return default_interface;
}


QWebHistoryInterface::QWebHistoryInterface(QObject* parent)
    : QObject(parent)
{
}

QWebHistoryInterface::~QWebHistoryInterface()
{
    if (default_interface == this)
        default_interface = 0;
}


