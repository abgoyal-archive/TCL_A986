
#if PLATFORM(ANDROID) && ENABLE(APPLICATION_INSTALLED)

#ifndef ApplicationInstalledCallback_h
#define ApplicationInstalledCallback_h

#include <wtf/Platform.h>
#include <wtf/RefCounted.h>

namespace WebCore {

class ApplicationInstalledCallback : public RefCounted<ApplicationInstalledCallback> {
public:
    virtual ~ApplicationInstalledCallback() { }
    virtual void handleEvent(bool isInstalled) = 0;
};

} // namespace WebCore

#endif // ApplicationInstalledCallback_h

#endif // PLATFORM(ANDROID) && ENABLE(APPLICATION_INSTALLED)
