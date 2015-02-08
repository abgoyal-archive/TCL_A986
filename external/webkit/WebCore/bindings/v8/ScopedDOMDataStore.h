

#ifndef ScopedDOMDataStore_h
#define ScopedDOMDataStore_h

#include "DOMDataStore.h"

namespace WebCore {

    // ScopedDOMDataStore
    //
    // ScopedDOMDataStore is a DOMDataStore that controls limits the lifetime of
    // the store to the lifetime of the object itself.  In other words, when the
    // ScopedDOMDataStore object is deallocated, the maps that belong to the store
    // are deallocated as well.
    //
    class ScopedDOMDataStore : public DOMDataStore {
    public:
        ScopedDOMDataStore(DOMData*);

        // This can be called when WTF thread is tearing down.
        // We assume that all child threads running V8 instances are created by WTF.
        virtual ~ScopedDOMDataStore();
    };

} // namespace WebCore

#endif // ScopedDOMDataStore_h
