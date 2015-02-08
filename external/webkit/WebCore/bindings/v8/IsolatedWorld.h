

#ifndef IsolatedWorld_h
#define IsolatedWorld_h

#include "DOMWrapperWorld.h"
#include "V8DOMMap.h"

namespace WebCore {

// An DOMWrapperWorld other than the thread's normal world.
class IsolatedWorld : public DOMWrapperWorld {
public:
    static PassRefPtr<IsolatedWorld> create() { return adoptRef(new IsolatedWorld()); }
    static int count() { return isolatedWorldCount; }

    DOMDataStore* domDataStore() const { return m_domDataStore.getStore(); }

protected:
    IsolatedWorld();
    ~IsolatedWorld();

private:
    // The backing store for the isolated world's DOM wrappers.  This class
    // doesn't have visibility into the wrappers.  This handle simply helps
    // manage their lifetime.
    DOMDataStoreHandle m_domDataStore;

    static int isolatedWorldCount;
};

} // namespace WebCore

#endif // IsolatedWorld_h
