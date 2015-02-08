

#ifndef StorageAreaProxy_h
#define StorageAreaProxy_h

#if ENABLE(DOM_STORAGE)

#include "StorageArea.h"

namespace WebKit { class WebStorageArea; }

namespace WebCore {

class Frame;
class SecurityOrigin;

class StorageAreaProxy : public StorageArea {
public:
    StorageAreaProxy(WebKit::WebStorageArea*, StorageType);
    virtual ~StorageAreaProxy();

    // The HTML5 DOM Storage API
    virtual unsigned length() const;
    virtual String key(unsigned index) const;
    virtual String getItem(const String& key) const;
    virtual String setItem(const String& key, const String& value, ExceptionCode& ec, Frame* sourceFrame);
    virtual String removeItem(const String& key, Frame* sourceFrame);
    virtual bool clear(Frame* sourceFrame);
    virtual bool contains(const String& key) const;

private:
    void storageEvent(const String& key, const String& oldValue, const String& newValue, StorageType, SecurityOrigin*, Frame* sourceFrame);

    OwnPtr<WebKit::WebStorageArea> m_storageArea;
    StorageType m_storageType;
};

} // namespace WebCore

#endif // ENABLE(DOM_STORAGE)

#endif // StorageAreaProxy_h
