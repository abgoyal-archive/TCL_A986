

#ifndef PositionError_h
#define PositionError_h

#include "PlatformString.h"
#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>

namespace WebCore {
    
class PositionError : public RefCounted<PositionError> {
public:
    enum ErrorCode {
        PERMISSION_DENIED = 1,
        POSITION_UNAVAILABLE = 2,
        TIMEOUT = 3
    };
    
    static PassRefPtr<PositionError> create(ErrorCode code, const String& message) { return adoptRef(new PositionError(code, message)); }

    ErrorCode code() const { return m_code; }
    const String& message() const { return m_message; }
    void setIsFatal(bool isFatal) { m_isFatal = isFatal; }
    bool isFatal() const { return m_isFatal; }
    
private:
    PositionError(ErrorCode code, const String& message) 
        : m_code(code)
        , m_message(message)
        , m_isFatal(false)
    {
    }
    
    ErrorCode m_code;
    String m_message;
    // Whether the error is fatal, such that no request can ever obtain a good
    // position fix in the future.
    bool m_isFatal;
};
    
} // namespace WebCore

#endif // PositionError_h
