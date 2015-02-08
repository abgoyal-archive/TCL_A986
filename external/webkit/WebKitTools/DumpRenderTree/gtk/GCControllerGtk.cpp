

#include "config.h"
#include "GCController.h"

#include <glib.h>
#include <webkit/webkit.h>

extern "C" {
extern void webkit_gc_collect_javascript_objects();
extern void webkit_gc_collect_javascript_objects_on_alternate_thread(gboolean waitUntilDone);
extern gsize webkit_gc_count_javascript_objects();
}

void GCController::collect() const
{
    webkit_gc_collect_javascript_objects();
}

void GCController::collectOnAlternateThread(bool waitUntilDone) const
{
    webkit_gc_collect_javascript_objects_on_alternate_thread(waitUntilDone);
}

size_t GCController::getJSObjectCount() const
{
    return webkit_gc_count_javascript_objects();
}
