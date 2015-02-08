

#ifndef ImageBufferData_h
#define ImageBufferData_h

#include "PlatformContextSkia.h"

#include "skia/ext/platform_canvas.h"

namespace WebCore {

class ImageBufferData {
public:
    ImageBufferData(const IntSize&);

    skia::PlatformCanvas m_canvas;
    PlatformContextSkia m_platformContext;
};

}  // namespace WebCore

#endif  // ImageBufferData_h
