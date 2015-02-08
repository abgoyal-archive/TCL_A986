

#include "config.h"
#include "V8DOMMap.h"

#include "DOMData.h"
#include "DOMDataStore.h"
#include "DOMObjectsInclude.h"
#include "MainThreadDOMData.h"
#include "ScopedDOMDataStore.h"

namespace WebCore {

DOMDataStoreHandle::DOMDataStoreHandle()
    : m_store(new ScopedDOMDataStore(DOMData::getCurrent()))
{
}

DOMDataStoreHandle::~DOMDataStoreHandle()
{
}

static bool fasterDOMStoreAccess = false;

static inline DOMDataStore& getDOMDataStore()
{
    if (LIKELY(fasterDOMStoreAccess)) {
        ASSERT(WTF::isMainThread());
        return MainThreadDOMData::getCurrentMainThreadStore();
    }

    return DOMData::getCurrent()->getStore();
}

void enableFasterDOMStoreAccess()
{
    fasterDOMStoreAccess = true;
}

DOMNodeMapping& getDOMNodeMap()
{
    return getDOMDataStore().domNodeMap();
}

DOMWrapperMap<void>& getDOMObjectMap()
{
    return getDOMDataStore().domObjectMap();
}

DOMWrapperMap<void>& getActiveDOMObjectMap()
{
    return getDOMDataStore().activeDomObjectMap();
}

#if ENABLE(SVG)

DOMWrapperMap<SVGElementInstance>& getDOMSVGElementInstanceMap()
{
    return getDOMDataStore().domSvgElementInstanceMap();
}

// Map of SVG objects with contexts to V8 objects
DOMWrapperMap<void>& getDOMSVGObjectWithContextMap()
{
    return getDOMDataStore().domSvgObjectWithContextMap();
}

#endif // ENABLE(SVG)

static void removeAllDOMObjectsInCurrentThreadHelper()
{
    v8::HandleScope scope;

    // Deref all objects in the delayed queue.
    DOMData::getCurrent()->derefDelayedObjects();

    // The DOM objects with the following types only exist on the main thread.
    if (WTF::isMainThread()) {
        // Remove all DOM nodes.
        DOMData::removeObjectsFromWrapperMap<Node>(getDOMNodeMap());

#if ENABLE(SVG)
        // Remove all SVG element instances in the wrapper map.
        DOMData::removeObjectsFromWrapperMap<SVGElementInstance>(getDOMSVGElementInstanceMap());

        // Remove all SVG objects with context in the wrapper map.
        DOMData::removeObjectsFromWrapperMap<void>(getDOMSVGObjectWithContextMap());
#endif
    }

    // Remove all DOM objects in the wrapper map.
    DOMData::removeObjectsFromWrapperMap<void>(getDOMObjectMap());

    // Remove all active DOM objects in the wrapper map.
    DOMData::removeObjectsFromWrapperMap<void>(getActiveDOMObjectMap());
}

void removeAllDOMObjectsInCurrentThread()
{
    // Use the locker only if it has already been invoked before, as by worker thread.
    if (v8::Locker::IsActive()) {
        v8::Locker locker;
        removeAllDOMObjectsInCurrentThreadHelper();
    } else
        removeAllDOMObjectsInCurrentThreadHelper();
}


void visitDOMNodesInCurrentThread(DOMWrapperMap<Node>::Visitor* visitor)
{
    v8::HandleScope scope;

    WTF::MutexLocker locker(DOMDataStore::allStoresMutex());
    DOMDataList& list = DOMDataStore::allStores();
    for (size_t i = 0; i < list.size(); ++i) {
        DOMDataStore* store = list[i];
        if (!store->domData()->owningThread() == WTF::currentThread())
            continue;

        store->domNodeMap().visit(visitor);
    }
}

void visitDOMObjectsInCurrentThread(DOMWrapperMap<void>::Visitor* visitor)
{
    v8::HandleScope scope;

    WTF::MutexLocker locker(DOMDataStore::allStoresMutex());
    DOMDataList& list = DOMDataStore::allStores();
    for (size_t i = 0; i < list.size(); ++i) {
        DOMDataStore* store = list[i];
        if (!store->domData()->owningThread() == WTF::currentThread())
            continue;

        store->domObjectMap().visit(visitor);
    }
}

void visitActiveDOMObjectsInCurrentThread(DOMWrapperMap<void>::Visitor* visitor)
{
    v8::HandleScope scope;

    WTF::MutexLocker locker(DOMDataStore::allStoresMutex());
    DOMDataList& list = DOMDataStore::allStores();
    for (size_t i = 0; i < list.size(); ++i) {
        DOMDataStore* store = list[i];
        if (!store->domData()->owningThread() == WTF::currentThread())
            continue;

        store->activeDomObjectMap().visit(visitor);
    }
}

#if ENABLE(SVG)

void visitDOMSVGElementInstancesInCurrentThread(DOMWrapperMap<SVGElementInstance>::Visitor* visitor)
{
    v8::HandleScope scope;

    WTF::MutexLocker locker(DOMDataStore::allStoresMutex());
    DOMDataList& list = DOMDataStore::allStores();
    for (size_t i = 0; i < list.size(); ++i) {
        DOMDataStore* store = list[i];
        if (!store->domData()->owningThread() == WTF::currentThread())
            continue;

        store->domSvgElementInstanceMap().visit(visitor);
    }
}

void visitSVGObjectsInCurrentThread(DOMWrapperMap<void>::Visitor* visitor)
{
    v8::HandleScope scope;

    WTF::MutexLocker locker(DOMDataStore::allStoresMutex());
    DOMDataList& list = DOMDataStore::allStores();
    for (size_t i = 0; i < list.size(); ++i) {
        DOMDataStore* store = list[i];
        if (!store->domData()->owningThread() == WTF::currentThread())
            continue;

        store->domSvgObjectWithContextMap().visit(visitor);
    }
}

#endif

} // namespace WebCore
