
#ifndef FormatDataStreamCurl_h
#define FormatDataStreamCurl_h

#include "config.h"

#include "FileSystem.h"
#include "ResourceHandle.h"
#include <stdio.h>

namespace WebCore {

class FormDataStream {
public:
    FormDataStream(ResourceHandle* handle)
        : m_resourceHandle(handle)
        , m_file(0)
        , m_formDataElementIndex(0)
        , m_formDataElementDataOffset(0)
    {
    }

    ~FormDataStream();

    size_t read(void* ptr, size_t blockSize, size_t numberOfBlocks);
    bool hasMoreElements() const;

private:
    // We can hold a weak reference to our ResourceHandle as it holds a strong reference
    // to us through its ResourceHandleInternal.
    ResourceHandle* m_resourceHandle;

    FILE* m_file;
    size_t m_formDataElementIndex;
    size_t m_formDataElementDataOffset;
};

} // namespace WebCore

#endif // FormDataStreamCurl_h
