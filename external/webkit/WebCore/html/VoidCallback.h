

#ifndef VoidCallback_h
#define VoidCallback_h

#include <wtf/RefCounted.h>

namespace WebCore {

class VoidCallback : public RefCounted<VoidCallback> {
public:
    virtual ~VoidCallback() { }
    
    virtual void handleEvent() = 0;
    
protected:
    VoidCallback() {}
};

} // namespace WebCore

#endif
