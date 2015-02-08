

#ifndef ImageBufferData_h
#define ImageBufferData_h

namespace WebCore {

    class IntSize;
    class ImageBufferData {
    public:
        ImageBufferData(const IntSize& size);
        RefPtr<SharedBitmap> m_bitmap;
    };

}  // namespace WebCore

#endif  // ImageBufferData_h
