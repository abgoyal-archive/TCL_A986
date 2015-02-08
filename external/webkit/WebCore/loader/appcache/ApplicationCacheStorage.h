

#ifndef ApplicationCacheStorage_h
#define ApplicationCacheStorage_h

#if ENABLE(OFFLINE_WEB_APPLICATIONS)

#include "PlatformString.h"
#include "SQLiteDatabase.h"
#include "StringHash.h"

#include <wtf/HashCountedSet.h>

namespace WebCore {

class ApplicationCache;
class ApplicationCacheHost;
class ApplicationCacheGroup;
class ApplicationCacheResource;
class KURL;
template <class T>
class StorageIDJournal;

class ApplicationCacheStorage : public Noncopyable {
public:
    void setCacheDirectory(const String&);
    const String& cacheDirectory() const;
    
    void setMaximumSize(int64_t size);
    int64_t maximumSize() const;
    bool isMaximumSizeReached() const;
    int64_t spaceNeeded(int64_t cacheToSave);

    ApplicationCacheGroup* cacheGroupForURL(const KURL&); // Cache to load a main resource from.
    ApplicationCacheGroup* fallbackCacheGroupForURL(const KURL&); // Cache that has a fallback entry to load a main resource from if normal loading fails.

    ApplicationCacheGroup* findOrCreateCacheGroup(const KURL& manifestURL);
    void cacheGroupDestroyed(ApplicationCacheGroup*);
    void cacheGroupMadeObsolete(ApplicationCacheGroup*);
        
    bool storeNewestCache(ApplicationCacheGroup*); // Updates the cache group, but doesn't remove old cache.
    bool store(ApplicationCacheResource*, ApplicationCache*);
    bool storeUpdatedType(ApplicationCacheResource*, ApplicationCache*);

    // Removes the group if the cache to be removed is the newest one (so, storeNewestCache() needs to be called beforehand when updating).
    void remove(ApplicationCache*);
    
    void empty();
    
    static bool storeCopyOfCache(const String& cacheDirectory, ApplicationCacheHost*);

    bool manifestURLs(Vector<KURL>* urls);
    bool cacheGroupSize(const String& manifestURL, int64_t* size);
    bool deleteCacheGroup(const String& manifestURL);
    void vacuumDatabaseFile();
private:
    ApplicationCacheStorage();
    PassRefPtr<ApplicationCache> loadCache(unsigned storageID);
    ApplicationCacheGroup* loadCacheGroup(const KURL& manifestURL);
    
    typedef StorageIDJournal<ApplicationCacheResource> ResourceStorageIDJournal;
    typedef StorageIDJournal<ApplicationCacheGroup> GroupStorageIDJournal;

    bool store(ApplicationCacheGroup*, GroupStorageIDJournal*);
    bool store(ApplicationCache*, ResourceStorageIDJournal*);
    bool store(ApplicationCacheResource*, unsigned cacheStorageID);

    void loadManifestHostHashes();
    
    void verifySchemaVersion();
    
    void openDatabase(bool createIfDoesNotExist);
    
    bool executeStatement(SQLiteStatement&);
    bool executeSQLCommand(const String&);

    void checkForMaxSizeReached();
    
    String m_cacheDirectory;
    String m_cacheFile;

    int64_t m_maximumSize;
    bool m_isMaximumSizeReached;

    SQLiteDatabase m_database;

    // In order to quickly determine if a given resource exists in an application cache,
    // we keep a hash set of the hosts of the manifest URLs of all non-obsolete cache groups.
    HashCountedSet<unsigned, AlreadyHashed> m_cacheHostSet;
    
    typedef HashMap<String, ApplicationCacheGroup*> CacheGroupMap;
    CacheGroupMap m_cachesInMemory; // Excludes obsolete cache groups.

    friend ApplicationCacheStorage& cacheStorage();
};
 
ApplicationCacheStorage& cacheStorage();
    
} // namespace WebCore

#endif // ENABLE(OFFLINE_WEB_APPLICATIONS)

#endif // ApplicationCacheStorage_h
