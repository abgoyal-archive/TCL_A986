

#include "config.h"
#include "GCControllerQt.h"

#include <qwebpage.h>

extern int qt_drt_javaScriptObjectsCount();
extern void qt_drt_garbageCollector_collect();

extern void qt_drt_garbageCollector_collectOnAlternateThread(bool waitUntilDone);

GCController::GCController(QWebPage* parent)
    : QObject(parent)
{
}

void GCController::collect() const
{
    qt_drt_garbageCollector_collect();
}

void GCController::collectOnAlternateThread(bool waitUntilDone) const
{
    qt_drt_garbageCollector_collectOnAlternateThread(waitUntilDone);
}

size_t GCController::getJSObjectCount() const
{
    return qt_drt_javaScriptObjectsCount();
}
