

#ifndef WebAccessibilityCacheImpl_h
#define WebAccessibilityCacheImpl_h

#include "AccessibilityObjectWrapper.h"
#include "WebAccessibilityCache.h"
#include <wtf/HashMap.h>
#include <wtf/PassRefPtr.h>

namespace WebKit {

// FIXME: Should be eliminated to use AXObjectCache instead.
class WebAccessibilityCacheImpl : public WebKit::WebAccessibilityCache {
public:
    virtual void initialize(WebView* view);
    virtual bool isInitialized() const { return m_initialized; }

    virtual WebAccessibilityObject getObjectById(int);
    virtual bool isValidId(int) const;
    virtual int addOrGetId(const WebKit::WebAccessibilityObject&);

    virtual void remove(int);
    virtual void clear();

protected:
    friend class WebKit::WebAccessibilityCache;

    WebAccessibilityCacheImpl();
    ~WebAccessibilityCacheImpl();

private:
    // FIXME: This can be just part of Chromium's AccessibilityObjectWrapper.
    class WeakHandle : public WebCore::AccessibilityObjectWrapper {
    public:
        static PassRefPtr<WeakHandle> create(WebCore::AccessibilityObject*);
        virtual void detach();
    private:
        WeakHandle(WebCore::AccessibilityObject*);
    };

    typedef HashMap<int, RefPtr<WeakHandle> > ObjectMap;
    typedef HashMap<WebCore::AccessibilityObject*, int> IdMap;

    // Hashmap for caching of elements in use by the AT, mapping id (int) to
    // WebAccessibilityObject.
    ObjectMap m_objectMap;
    // Hashmap for caching of elements in use by the AT, mapping a
    // AccessibilityObject pointer to its id (int). Needed for reverse lookup,
    // to ensure unnecessary duplicate entries are not created in the
    // ObjectMap and for focus changes in WebKit.
    IdMap m_idMap;

    // Unique identifier for retrieving an accessibility object from the page's
    // hashmaps. Id is always 0 for the root of the accessibility object
    // hierarchy (on a per-renderer process basis).
    int m_nextNewId;

    bool m_initialized;
};

}

#endif
