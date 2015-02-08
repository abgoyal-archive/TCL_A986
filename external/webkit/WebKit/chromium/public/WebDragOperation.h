

#ifndef WebDragOperation_h
#define WebDragOperation_h

#include <limits.h>

namespace WebKit {

// "Verb" of a drag-and-drop operation as negotiated between the source and
// destination.
// (These constants match their equivalents in WebCore's DragActions.h and
// should not be renumbered.)
enum WebDragOperation {
    WebDragOperationNone    = 0,
    WebDragOperationCopy    = 1,
    WebDragOperationLink    = 2,
    WebDragOperationGeneric = 4,
    WebDragOperationPrivate = 8,
    WebDragOperationMove    = 16,
    WebDragOperationDelete  = 32,
    WebDragOperationEvery   = UINT_MAX
};

// Alternate typedef to make it clear when this is being used as a mask
// with potentially multiple value bits set.
typedef WebDragOperation WebDragOperationsMask;

} // namespace WebKit

#endif
