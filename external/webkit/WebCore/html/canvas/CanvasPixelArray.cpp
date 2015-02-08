

#include "config.h"
#include "CanvasPixelArray.h"

namespace WebCore {
    
    PassRefPtr<CanvasPixelArray> CanvasPixelArray::create(unsigned length)
    {
        return adoptRef(new CanvasPixelArray(length));
    }
    
    CanvasPixelArray::CanvasPixelArray(unsigned length)
        : m_data(WTF::ByteArray::create(length))
    {
    }
    
}
