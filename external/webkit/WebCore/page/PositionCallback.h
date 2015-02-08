

#ifndef PositionCallback_h
#define PositionCallback_h

#include <wtf/Platform.h>
#include <wtf/RefCounted.h>

namespace WebCore {
    
    class Geoposition;
    
    class PositionCallback : public RefCounted<PositionCallback> {
    public:
        virtual ~PositionCallback() { }
        virtual void handleEvent(Geoposition*) = 0;
    };
    
} // namespace WebCore

#endif // PositionCallback_h
