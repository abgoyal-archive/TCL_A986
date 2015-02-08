

#ifndef WebStorageAreaImpl_h
#define WebStorageAreaImpl_h

#if ENABLE(DOM_STORAGE)

#include "StorageAreaImpl.h"
#include "WebStorageArea.h"

namespace WebKit {

class WebStorageAreaImpl : public WebStorageArea {
public:
    WebStorageAreaImpl(PassRefPtr<WebCore::StorageArea> storageArea);
    virtual ~WebStorageAreaImpl();
    virtual unsigned length();
    virtual WebString key(unsigned index);
    virtual WebString getItem(const WebString& key);
    virtual void setItem(const WebString& key, const WebString& value, const WebURL& url, Result& result, WebString& oldValue);
    virtual void removeItem(const WebString& key, const WebURL& url, WebString& oldValue);
    virtual void clear(const WebURL& url, bool& somethingCleared);

    // For storage events in single-process mode and test shell.
    static const WebURL* currentStorageEventURL() { return storageEventURL; }

private:
    class ScopedStorageEventURL {
    public:
        ScopedStorageEventURL(const WebURL& url)
        {
            // FIXME: Once storage events are fired async in WebKit (as they should
            //        be) this can be ASSERTed to be 0 rather than saved.
            m_existingStorageEventURL = storageEventURL;
            storageEventURL = &url;
        }
        ~ScopedStorageEventURL()
        {
            storageEventURL = m_existingStorageEventURL;
        }

    private:
        const WebURL* m_existingStorageEventURL;
    };

    static const WebURL* storageEventURL;

    RefPtr<WebCore::StorageArea> m_storageArea;
};

} // namespace WebKit

#endif // ENABLE(DOM_STORAGE)

#endif // WebStorageAreaImpl_h
