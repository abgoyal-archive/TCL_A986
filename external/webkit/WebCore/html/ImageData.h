

#ifndef ImageData_h
#define ImageData_h

#include "CanvasPixelArray.h"
#include <wtf/RefCounted.h>
#include <wtf/RefPtr.h>

namespace WebCore {

    class ImageData : public RefCounted<ImageData> {
    public:
        static PassRefPtr<ImageData> create(unsigned width, unsigned height);

        unsigned width() const { return m_width; }
        unsigned height() const { return m_height; }
        CanvasPixelArray* data() const { return m_data.get(); }

    private:
        ImageData(unsigned width, unsigned height);
        unsigned m_width;
        unsigned m_height;
        RefPtr<CanvasPixelArray> m_data;
    };

} // namespace WebCore

#endif // ImageData_h
