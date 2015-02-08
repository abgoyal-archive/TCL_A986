

#ifndef StaticDOMDataStore_h
#define StaticDOMDataStore_h

#include "DOMDataStore.h"

namespace WebCore {

// StaticDOMDataStore
//
// StaticDOMDataStore is a DOMDataStore that manages the lifetime of the store
// statically.  This encapsulates thread-specific DOM data for the main
// thread.  All the maps in it are static.  This is because we are unable to
// rely on WTF::ThreadSpecificThreadExit to do the cleanup since the place that
// tears down the main thread can not call any WTF functions.
//
class StaticDOMDataStore : public DOMDataStore {
public:
    StaticDOMDataStore(DOMData*);

private:
    IntrusiveDOMWrapperMap m_staticDomNodeMap;
    InternalDOMWrapperMap<void> m_staticDomObjectMap;
    InternalDOMWrapperMap<void> m_staticActiveDomObjectMap;
#if ENABLE(SVG)
    InternalDOMWrapperMap<SVGElementInstance> m_staticDomSvgElementInstanceMap;
    InternalDOMWrapperMap<void> m_staticDomSvgObjectWithContextMap;
#endif
};

} // namespace WebCore

#endif // StaticDOMDataStore_h

