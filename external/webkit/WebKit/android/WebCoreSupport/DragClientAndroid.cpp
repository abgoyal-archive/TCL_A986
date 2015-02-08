

#define LOG_TAG "WebCore"

#include "config.h"
#include "DragClientAndroid.h"
#include "NotImplemented.h"

namespace android {

void DragClientAndroid::dragControllerDestroyed() { notImplemented(); delete this; }

void DragClientAndroid::willPerformDragDestinationAction(DragDestinationAction, DragData*) { notImplemented(); }

DragDestinationAction DragClientAndroid::actionMaskForDrag(DragData*) { notImplemented(); return DragDestinationActionNone; }

DragSourceAction DragClientAndroid::dragSourceActionMaskForPoint(const IntPoint&) { notImplemented(); return DragSourceActionNone; }

void* DragClientAndroid::createDragImageForLink(KURL&, String const&, Frame*) { return NULL; }
void DragClientAndroid::willPerformDragSourceAction(DragSourceAction, IntPoint const&, Clipboard*) {}
void DragClientAndroid::startDrag(void*, IntPoint const&, IntPoint const&, Clipboard*, Frame*, bool) {}

}
