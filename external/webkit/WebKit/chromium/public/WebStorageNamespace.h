

#ifndef WebStorageNamespace_h
#define WebStorageNamespace_h

#include "WebCommon.h"

namespace WebKit {

class WebStorageArea;
class WebString;

// WebStorageNamespace represents a collection of StorageAreas. Typically, you'll have
// multiple StorageNamespaces to represent the SessionStorage for each tab and a single
// StorageNamespace to represent LocalStorage for the entire browser.
class WebStorageNamespace {
public:
    // Create a new WebStorageNamespace. LocalStorageNamespaces require a path to specify
    // where the SQLite databases that make LocalStorage data persistent are located.
    // If path is empty, data will not persist. You should call delete on the returned
    // object when you're finished.
    WEBKIT_API static WebStorageNamespace* createLocalStorageNamespace(const WebString& backingDirectoryPath, unsigned quota);
    WEBKIT_API static WebStorageNamespace* createSessionStorageNamespace();

    static const unsigned noQuota = UINT_MAX;

    virtual ~WebStorageNamespace() { }

    // Create a new WebStorageArea object. Two subsequent calls with the same origin
    // will return two different WebStorageArea objects that share the same backing store.
    // You should call delete on the returned object when you're finished.
    virtual WebStorageArea* createStorageArea(const WebString& origin) = 0;

    // Copy a StorageNamespace. This only makes sense in the case of SessionStorage.
    virtual WebStorageNamespace* copy() = 0;

    // Shutdown the StorageNamespace. Write all StorageArea's to disk and disallow new
    // write activity.
    virtual void close() = 0;
};

} // namespace WebKit

#endif // WebStorageNamespace_h
