

#include "config.h"
#include "DragData.h"

#if ENABLE(DRAG_SUPPORT)
namespace WebCore {

#if !PLATFORM(MAC)
DragData::DragData(DragDataRef data, const IntPoint& clientPosition, const IntPoint& globalPosition, 
    DragOperation sourceOperationMask)
    : m_clientPosition(clientPosition)
    , m_globalPosition(globalPosition)
    , m_platformDragData(data)
    , m_draggingSourceOperationMask(sourceOperationMask)
{  
}
#endif

} // namespace WebCore

#endif // ENABLE(DRAG_SUPPORT)
