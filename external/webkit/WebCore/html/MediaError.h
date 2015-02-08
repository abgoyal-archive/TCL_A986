

#ifndef MediaError_h
#define MediaError_h

#if ENABLE(VIDEO)

#include <wtf/RefCounted.h>

namespace WebCore {

class MediaError : public RefCounted<MediaError> {
public:
    enum Code { MEDIA_ERR_ABORTED = 1, MEDIA_ERR_NETWORK, MEDIA_ERR_DECODE, MEDIA_ERR_SRC_NOT_SUPPORTED };

    static PassRefPtr<MediaError> create(Code code) { return adoptRef(new MediaError(code)); }

    Code code() const { return m_code; }

private:
    MediaError(Code code) : m_code(code) { }

    Code m_code;
};

} // namespace WebCore

#endif
#endif
