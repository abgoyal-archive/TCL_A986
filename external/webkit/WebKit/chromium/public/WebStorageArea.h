

#ifndef WebStorageArea_h
#define WebStorageArea_h

#include "WebCommon.h"
#include "WebString.h"

namespace WebKit {

class WebURL;

// In WebCore, there's one distinct StorageArea per origin per StorageNamespace. This
// class wraps a StorageArea.  All the methods have obvious connections to the spec:
// http://dev.w3.org/html5/webstorage/
class WebStorageArea {
public:
    virtual ~WebStorageArea() { }

    enum Result {
        ResultOK = 0,
        ResultBlockedByQuota,
        ResultBlockedByPolicy
    };

    // The number of key/value pairs in the storage area.
    virtual unsigned length() = 0;

    // Get a value for a specific key. Valid key indices are 0 through length() - 1.
    // Indexes may change on any set/removeItem call. Will return null if the index
    // provided is out of range.
    virtual WebString key(unsigned index) = 0;

    // Get the value that corresponds to a specific key. This returns null if there is
    // no entry for that key.
    virtual WebString getItem(const WebString& key) = 0;

    // Set the value that corresponds to a specific key. Result will either be ResultOK
    // or some particular error. The value is NOT set when there's an error.  url is the
    // url that should be used if a storage event fires.
    virtual void setItem(const WebString& key, const WebString& newValue, const WebURL& url, Result& result, WebString& oldValue)
    {
        bool quotaException = false;
        setItem(key, newValue, url, quotaException, oldValue);
        result = quotaException ? ResultBlockedByQuota : ResultOK;
    }
    // FIXME: Remove soon (once Chrome has rolled past this revision).
    virtual void setItem(const WebString& key, const WebString& newValue, const WebURL& url, bool& quotaException, WebString& oldValue)
    {
        Result result;
        setItem(key, newValue, url, result, oldValue);
        quotaException = result != ResultOK;
    }

    // Remove the value associated with a particular key.  url is the url that should be used
    // if a storage event fires.
    virtual void removeItem(const WebString& key, const WebURL& url, WebString& oldValue) = 0;

    // Clear all key/value pairs.  url is the url that should be used if a storage event fires.
    virtual void clear(const WebURL& url, bool& somethingCleared) = 0;
};

} // namespace WebKit

#endif // WebStorageArea_h
