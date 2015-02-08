

#include "config.h"

#if ENABLE(3D_CANVAS)

#include "CanvasNumberArray.h"
#include "PlatformString.h"

namespace WebCore {
    
PassRefPtr<CanvasNumberArray> CanvasNumberArray::create(unsigned length)
{
    return adoptRef(new CanvasNumberArray(length));
}

CanvasNumberArray::CanvasNumberArray(unsigned length)
    : m_data(length)
{
}

}

#endif // ENABLE(3D_CANVAS)
