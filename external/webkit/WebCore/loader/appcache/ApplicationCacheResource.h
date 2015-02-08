

#ifndef ApplicationCacheResource_h
#define ApplicationCacheResource_h

#if ENABLE(OFFLINE_WEB_APPLICATIONS)

#include "SubstituteResource.h"

namespace WebCore {

class ApplicationCacheResource : public SubstituteResource {
public:
    enum Type {
        Master = 1 << 0,
        Manifest = 1 << 1,
        Explicit = 1 << 2,
        Foreign = 1 << 3,
        Fallback = 1 << 4
    };
        
    static PassRefPtr<ApplicationCacheResource> create(const KURL& url, const ResourceResponse& response, unsigned type, PassRefPtr<SharedBuffer> buffer = SharedBuffer::create())
    {
        ASSERT(!url.hasFragmentIdentifier());
        return adoptRef(new ApplicationCacheResource(url, response, type, buffer));
    }

    unsigned type() const { return m_type; }
    void addType(unsigned type);
    
    void setStorageID(unsigned storageID) { m_storageID = storageID; }
    unsigned storageID() const { return m_storageID; }
    void clearStorageID() { m_storageID = 0; }
    int64_t estimatedSizeInStorage();

#ifndef NDEBUG
    static void dumpType(unsigned type);
#endif
    
private:
    ApplicationCacheResource(const KURL& url, const ResourceResponse& response, unsigned type, PassRefPtr<SharedBuffer> buffer);

    unsigned m_type;
    unsigned m_storageID;
    int64_t m_estimatedSizeInStorage;
};
    
} // namespace WebCore

#endif // ENABLE(OFFLINE_WEB_APPLICATIONS)

#endif // ApplicationCacheResource_h
