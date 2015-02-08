

#ifndef ApplicationCache_h
#define ApplicationCache_h

#if ENABLE(OFFLINE_WEB_APPLICATIONS)

#include "PlatformString.h"
#include "StringHash.h"
#include <wtf/HashMap.h>
#include <wtf/HashSet.h>
#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>

namespace WebCore {

class ApplicationCacheGroup;
class ApplicationCacheResource;
class DocumentLoader;
class KURL;

class ResourceRequest;

typedef Vector<std::pair<KURL, KURL> > FallbackURLVector;

class ApplicationCache : public RefCounted<ApplicationCache> {
public:
    static PassRefPtr<ApplicationCache> create() { return adoptRef(new ApplicationCache); }
    ~ApplicationCache();

    void addResource(PassRefPtr<ApplicationCacheResource> resource);
    unsigned removeResource(const String& url);
    
    void setManifestResource(PassRefPtr<ApplicationCacheResource> manifest);
    ApplicationCacheResource* manifestResource() const { return m_manifest; }
    
    void setGroup(ApplicationCacheGroup*);
    ApplicationCacheGroup* group() const { return m_group; }

    bool isComplete() const;

    ApplicationCacheResource* resourceForRequest(const ResourceRequest&);
    ApplicationCacheResource* resourceForURL(const String& url);

    void setAllowsAllNetworkRequests(bool value) { m_allowAllNetworkRequests = value; }
    bool allowsAllNetworkRequests() const { return m_allowAllNetworkRequests; }
    void setOnlineWhitelist(const Vector<KURL>& onlineWhitelist);
    const Vector<KURL>& onlineWhitelist() const { return m_onlineWhitelist; }
    bool isURLInOnlineWhitelist(const KURL&); // There is an entry in online whitelist that has the same origin as the resource's URL and that is a prefix match for the resource's URL.

    void setFallbackURLs(const FallbackURLVector&);
    const FallbackURLVector& fallbackURLs() const { return m_fallbackURLs; }
    bool urlMatchesFallbackNamespace(const KURL&, KURL* fallbackURL = 0);
    
#ifndef NDEBUG
    void dump();
#endif

    typedef HashMap<String, RefPtr<ApplicationCacheResource> > ResourceMap;
    ResourceMap::const_iterator begin() const { return m_resources.begin(); }
    ResourceMap::const_iterator end() const { return m_resources.end(); }
    
    void setStorageID(unsigned storageID) { m_storageID = storageID; }
    unsigned storageID() const { return m_storageID; }
    void clearStorageID();
    
    static bool requestIsHTTPOrHTTPSGet(const ResourceRequest&);

    int64_t estimatedSizeInStorage() const { return m_estimatedSizeInStorage; }

private:
    ApplicationCache();
    
    ApplicationCacheGroup* m_group;
    ResourceMap m_resources;
    ApplicationCacheResource* m_manifest;

    bool m_allowAllNetworkRequests;
    Vector<KURL> m_onlineWhitelist;
    FallbackURLVector m_fallbackURLs;

    // The total size of the resources belonging to this Application Cache instance.
    // This is an estimation of the size this Application Cache occupies in the
    // database file.
    int64_t m_estimatedSizeInStorage;

    unsigned m_storageID;
};

} // namespace WebCore

#endif // ENABLE(OFFLINE_WEB_APPLICATIONS)

#endif // ApplicationCache_h
