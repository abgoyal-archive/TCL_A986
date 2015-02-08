

#ifndef CachePolicy_h
#define CachePolicy_h

namespace WebCore {

    enum CachePolicy {
        CachePolicyCache,
        CachePolicyVerify,
        CachePolicyRevalidate,
        CachePolicyReload,
        CachePolicyAllowStale
    };

}

#endif
