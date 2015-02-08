

#ifndef GeolocationError_h
#define GeolocationError_h

#if ENABLE(CLIENT_BASED_GEOLOCATION)

#include "PlatformString.h"
#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>
#include <wtf/RefPtr.h>

namespace WebCore {

class GeolocationError : public RefCounted<GeolocationError> {
public:
    enum ErrorCode {
        PermissionDenied,
        PositionUnavailable
    };

    static PassRefPtr<GeolocationError> create(ErrorCode code, const String& message) { return adoptRef(new GeolocationError(code, message)); }

    ErrorCode code() const { return m_code; }
    const String& message() const { return m_message; }

private:
    GeolocationError(ErrorCode code, const String& message)
        : m_code(code)
        , m_message(message)
    {
    }

    ErrorCode m_code;
    String m_message; 
};

} // namespace WebCore

#endif // ENABLE(CLIENT_BASED_GEOLOCATION)

#endif // GeolocationError_h
