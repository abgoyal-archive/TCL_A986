

#ifndef DragActions_h
#define DragActions_h

#include <limits.h>

namespace WebCore {

    // WebCoreDragDestinationAction should be kept in sync with WebDragDestinationAction
    typedef enum {
        DragDestinationActionNone    = 0,
        DragDestinationActionDHTML   = 1,
        DragDestinationActionEdit    = 2,
        DragDestinationActionLoad    = 4,
        DragDestinationActionAny     = UINT_MAX
    } DragDestinationAction;
    
    // WebCoreDragSourceAction should be kept in sync with WebDragSourceAction
    typedef enum {
        DragSourceActionNone         = 0,
        DragSourceActionDHTML        = 1,
        DragSourceActionImage        = 2,
        DragSourceActionLink         = 4,
        DragSourceActionSelection    = 8,
        DragSourceActionAny          = UINT_MAX
    } DragSourceAction;
    
    //matches NSDragOperation
    typedef enum {
        DragOperationNone    = 0,
        DragOperationCopy    = 1,
        DragOperationLink    = 2,
        DragOperationGeneric = 4,
        DragOperationPrivate = 8,
        DragOperationMove    = 16,
        DragOperationDelete  = 32,
        DragOperationEvery   = UINT_MAX
    } DragOperation;
    
}

#endif // !DragActions_h
