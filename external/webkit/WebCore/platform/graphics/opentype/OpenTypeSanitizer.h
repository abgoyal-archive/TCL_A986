

#ifndef OpenTypeSanitizer_h
#define OpenTypeSanitizer_h

#if ENABLE(OPENTYPE_SANITIZER)
#include <wtf/Forward.h>

namespace WebCore {

class SharedBuffer;

class OpenTypeSanitizer {
public:
    explicit OpenTypeSanitizer(SharedBuffer* buffer)
        : m_buffer(buffer)
    {
    }

    PassRefPtr<SharedBuffer> sanitize();

private:
    SharedBuffer* const m_buffer;
};

} // namespace WebCore

#endif // ENABLE(OPENTYPE_SANITIZER)
#endif // OpenTypeSanitizer_h
