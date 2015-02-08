

#ifndef ImageBufferData_h
#define ImageBufferData_h

namespace WebCore {

class IntSize;

class ImageBufferData {
public:
    ImageBufferData(const IntSize&);

    void* m_data;
};

}  // namespace WebCore

#endif  // ImageBufferData_h
