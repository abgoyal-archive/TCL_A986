

#ifndef PositionErrorCallback_h
#define PositionErrorCallback_h

#include <wtf/Platform.h>
#include <wtf/RefCounted.h>

namespace WebCore {

    class PositionError;

    class PositionErrorCallback : public RefCounted<PositionErrorCallback> {
    public:
        virtual ~PositionErrorCallback() { }
        virtual void handleEvent(PositionError*) = 0;
    };
    
} // namespace WebCore

#endif // PositionErrorCallback_h
