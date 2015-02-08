

#include "config.h"
#include "WebAccessibilityCacheImpl.h"

#include "AccessibilityObject.h"
#include "AXObjectCache.h"
#include "Document.h"
#include "Frame.h"

#include "WebAccessibilityObject.h"
#include "WebFrameImpl.h"
#include "WebViewImpl.h"

using namespace WebCore;

namespace WebKit {

const int invalidObjectId = -1;
const int firstObjectId = 1000;

static PassRefPtr<AccessibilityObject> toAccessibilityObject(const WebAccessibilityObject& object)
{
    return object;
}

// WebView ----------------------------------------------------------------

WebAccessibilityCache* WebAccessibilityCache::create()
{
    return new WebAccessibilityCacheImpl();
}

// WeakHandle -------------------------------------------------------------

PassRefPtr<WebAccessibilityCacheImpl::WeakHandle> WebAccessibilityCacheImpl::WeakHandle::create(AccessibilityObject* object)
{
    // FIXME: Remove resetting ref-count from AccessibilityObjectWrapper
    // and convert to use adoptRef.
    return new WebAccessibilityCacheImpl::WeakHandle(object);
}

WebAccessibilityCacheImpl::WeakHandle::WeakHandle(AccessibilityObject* object)
    : AccessibilityObjectWrapper(object)
{
    m_object->setWrapper(this);
}

// WebAccessibilityCacheImpl ----------------------------------------

void WebAccessibilityCacheImpl::WeakHandle::detach()
{
    if (m_object)
        m_object = 0;
}

WebAccessibilityCacheImpl::WebAccessibilityCacheImpl()
    : m_nextNewId(firstObjectId)
    , m_initialized(false)
{
}

WebAccessibilityCacheImpl::~WebAccessibilityCacheImpl()
{
}

void WebAccessibilityCacheImpl::initialize(WebView* view)
{
    AXObjectCache::enableAccessibility();
    WebAccessibilityObject root = view->accessibilityObject();
    if (root.isNull())
        return;

    RefPtr<AccessibilityObject> rootObject = toAccessibilityObject(root);

    // Insert root in hashmaps.
    m_objectMap.set(m_nextNewId, WeakHandle::create(rootObject.get()));
    m_idMap.set(rootObject.get(), m_nextNewId++);

    m_initialized = true;
}

WebAccessibilityObject WebAccessibilityCacheImpl::getObjectById(int id)
{
    ObjectMap::iterator it = m_objectMap.find(id);

    if (it == m_objectMap.end() || !it->second)
        return WebAccessibilityObject();

    return WebAccessibilityObject(it->second->accessibilityObject());
}

bool WebAccessibilityCacheImpl::isValidId(int id) const
{
    return id >= firstObjectId;
}

void WebAccessibilityCacheImpl::remove(int id)
{
    ObjectMap::iterator it = m_objectMap.find(id);

    if (it == m_objectMap.end())
        return;

    if (it->second) {
        // Erase element from reverse hashmap.
        IdMap::iterator it2 = m_idMap.find(it->second->accessibilityObject());
        if (it2 != m_idMap.end())
            m_idMap.remove(it2);
    }

    m_objectMap.remove(it);
}

void WebAccessibilityCacheImpl::clear()
{
    m_objectMap.clear();
    m_idMap.clear();
}

int WebAccessibilityCacheImpl::addOrGetId(const WebAccessibilityObject& object)
{
    if (object.isNull())
        return invalidObjectId;

    RefPtr<AccessibilityObject> o = toAccessibilityObject(object);

    IdMap::iterator it = m_idMap.find(o.get());

    if (it != m_idMap.end())
        return it->second;

    // Insert new accessibility object in hashmaps and return its newly
    // assigned accessibility object id.
    m_objectMap.set(m_nextNewId, WeakHandle::create(o.get()));
    m_idMap.set(o.get(), m_nextNewId);

    return m_nextNewId++;
}

}
