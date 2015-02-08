

#ifndef DocLoader_h
#define DocLoader_h

#include "CachedResource.h"
#include "CachedResourceHandle.h"
#include "CachePolicy.h"
#include "StringHash.h"
#include <wtf/HashMap.h>
#include <wtf/HashSet.h>
#include <wtf/ListHashSet.h>

namespace WebCore {

class CachedCSSStyleSheet;
class CachedFont;
class CachedImage;
class CachedScript;
class CachedXSLStyleSheet;
class Document;
class Frame;
class ImageLoader;
class KURL;

// The DocLoader manages the loading of scripts/images/stylesheets for a single document.
class DocLoader : public Noncopyable
{
friend class Cache;
friend class ImageLoader;

public:
    DocLoader(Document*);
    ~DocLoader();

    CachedImage* requestImage(const String& url);
    CachedCSSStyleSheet* requestCSSStyleSheet(const String& url, const String& charset);
    CachedCSSStyleSheet* requestUserCSSStyleSheet(const String& url, const String& charset);
    CachedScript* requestScript(const String& url, const String& charset);
    CachedFont* requestFont(const String& url);

#if ENABLE(XSLT)
    CachedXSLStyleSheet* requestXSLStyleSheet(const String& url);
#endif
#if ENABLE(XBL)
    CachedXBLDocument* requestXBLDocument(const String &url);
#endif

    // Logs an access denied message to the console for the specified URL.
    void printAccessDeniedMessage(const KURL& url) const;

    CachedResource* cachedResource(const String& url) const { return m_documentResources.get(url).get(); }
    
    typedef HashMap<String, CachedResourceHandle<CachedResource> > DocumentResourceMap;
    const DocumentResourceMap& allCachedResources() const { return m_documentResources; }

    bool autoLoadImages() const { return m_autoLoadImages; }
    void setAutoLoadImages(bool);
    
#ifdef ANDROID_BLOCK_NETWORK_IMAGE
    bool blockNetworkImage() const { return m_blockNetworkImage; }
    void setBlockNetworkImage(bool);
    bool shouldBlockNetworkImage(const String& url) const;
#endif

    CachePolicy cachePolicy() const;
    
    Frame* frame() const; // Can be NULL
    Document* doc() const { return m_doc; }

    void removeCachedResource(CachedResource*) const;

    void setLoadInProgress(bool);
    bool loadInProgress() const { return m_loadInProgress; }
    
    void setAllowStaleResources(bool allowStaleResources) { m_allowStaleResources = allowStaleResources; }

    void incrementRequestCount();
    void decrementRequestCount();
    int requestCount();
    
    void clearPreloads();
    void clearPendingPreloads();
    void preload(CachedResource::Type, const String& url, const String& charset, bool referencedFromBody);
    void checkForPendingPreloads();
    void printPreloadStats();
    
private:
    CachedResource* requestResource(CachedResource::Type, const String& url, const String& charset, bool isPreload = false);
    void requestPreload(CachedResource::Type, const String& url, const String& charset);

    void checkForReload(const KURL&);
    void checkCacheObjectStatus(CachedResource*);
    bool canRequest(CachedResource::Type, const KURL&);
    
    Cache* m_cache;
    HashSet<String> m_reloadedURLs;
    mutable DocumentResourceMap m_documentResources;
    Document* m_doc;
    
    int m_requestCount;
    
    ListHashSet<CachedResource*> m_preloads;
    struct PendingPreload {
        CachedResource::Type m_type;
        String m_url;
        String m_charset;
    };
    Vector<PendingPreload> m_pendingPreloads;
    
    //29 bits left
#ifdef ANDROID_BLOCK_NETWORK_IMAGE
    bool m_blockNetworkImage : 1;
#endif
    bool m_autoLoadImages : 1;
    bool m_loadInProgress : 1;
    bool m_allowStaleResources : 1;
};

}

#endif
