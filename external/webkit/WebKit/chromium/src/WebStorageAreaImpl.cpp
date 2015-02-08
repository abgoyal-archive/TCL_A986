

#include "config.h"
#include "WebStorageAreaImpl.h"

#if ENABLE(DOM_STORAGE)

#include "ExceptionCode.h"

#include "WebString.h"
#include "WebURL.h"

namespace WebKit {

const WebURL* WebStorageAreaImpl::storageEventURL = 0;

WebStorageAreaImpl::WebStorageAreaImpl(PassRefPtr<WebCore::StorageArea> storageArea)
    : m_storageArea(storageArea)
{
}

WebStorageAreaImpl::~WebStorageAreaImpl()
{
}

unsigned WebStorageAreaImpl::length()
{
    return m_storageArea->length();
}

WebString WebStorageAreaImpl::key(unsigned index)
{
    return m_storageArea->key(index);
}

WebString WebStorageAreaImpl::getItem(const WebString& key)
{
    return m_storageArea->getItem(key);
}

void WebStorageAreaImpl::setItem(const WebString& key, const WebString& value, const WebURL& url, Result& result, WebString& oldValue)
{
    int exceptionCode = 0;

    ScopedStorageEventURL scope(url);
    oldValue = m_storageArea->setItem(key, value, exceptionCode, 0);

    if (exceptionCode) {
        ASSERT(exceptionCode == WebCore::QUOTA_EXCEEDED_ERR);
        result = ResultBlockedByQuota;
    } else
        result = ResultOK;
}

void WebStorageAreaImpl::removeItem(const WebString& key, const WebURL& url, WebString& oldValue)
{
    ScopedStorageEventURL scope(url);
    oldValue = m_storageArea->removeItem(key, 0);
}

void WebStorageAreaImpl::clear(const WebURL& url, bool& somethingCleared)
{
    ScopedStorageEventURL scope(url);
    somethingCleared = m_storageArea->clear(0);
}

} // namespace WebKit

#endif // ENABLE(DOM_STORAGE)
