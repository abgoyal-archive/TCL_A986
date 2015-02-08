

#ifndef DOMWrapperWorld_h
#define DOMWrapperWorld_h

#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>
#include <wtf/RefPtr.h>

namespace WebCore {

// This class represent a collection of DOM wrappers for a specific world.
// The base class is pretty boring because the wrappers are actually stored
// statically in V8DOMMap and garbage collected by V8 itself.
class DOMWrapperWorld : public RefCounted<DOMWrapperWorld> {
public:
    static PassRefPtr<DOMWrapperWorld> create() { return adoptRef(new DOMWrapperWorld()); }

protected:
    DOMWrapperWorld();
};

DOMWrapperWorld* mainThreadNormalWorld();

} // namespace WebCore

#endif // DOMWrapperWorld_h
