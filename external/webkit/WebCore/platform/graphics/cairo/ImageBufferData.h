

#ifndef ImageBufferData_h
#define ImageBufferData_h

#include "cairo.h"

namespace WebCore {

class IntSize;

class ImageBufferData {
public:
    ImageBufferData(const IntSize&);

    cairo_surface_t* m_surface;
};

}  // namespace WebCore

#endif  // ImageBufferData_h
