

#include "KURLHash.h"
#include "StringHash.h"
#include <wtf/HashMap.h>
#include <wtf/HashSet.h>

namespace WebCore {

    class HTTPHeaderMap;
    class ResourceResponse;

    class CrossOriginPreflightResultCacheItem : public Noncopyable {
    public:
        CrossOriginPreflightResultCacheItem(bool credentials)
            : m_absoluteExpiryTime(0)
            , m_credentials(credentials)
        {
        }

        bool parse(const ResourceResponse&);
        bool allowsCrossOriginMethod(const String&) const;
        bool allowsCrossOriginHeaders(const HTTPHeaderMap&) const;
        bool allowsRequest(bool includeCredentials, const String& method, const HTTPHeaderMap& requestHeaders) const;

    private:
        typedef HashSet<String, CaseFoldingHash> HeadersSet;

        // FIXME: A better solution to holding onto the absolute expiration time might be
        // to start a timer for the expiration delta that removes this from the cache when
        // it fires.
        double m_absoluteExpiryTime;
        bool m_credentials;
        HashSet<String> m_methods;
        HeadersSet m_headers;
    };

    class CrossOriginPreflightResultCache : public Noncopyable {
    public:
        static CrossOriginPreflightResultCache& shared();

        void appendEntry(const String& origin, const KURL&, CrossOriginPreflightResultCacheItem*);
        bool canSkipPreflight(const String& origin, const KURL&, bool includeCredentials, const String& method, const HTTPHeaderMap& requestHeaders);

        void empty();

    private:
        CrossOriginPreflightResultCache() { }

        typedef HashMap<std::pair<String, KURL>, CrossOriginPreflightResultCacheItem*> CrossOriginPreflightResultHashMap;

        CrossOriginPreflightResultHashMap m_preflightHashMap;
    };

} // namespace WebCore
