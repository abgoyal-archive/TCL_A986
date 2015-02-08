

#import "WebResourcePrivate.h"
#import <wtf/PassRefPtr.h>

#if defined(BUILDING_ON_TIGER) || defined(BUILDING_ON_LEOPARD)
#define MAIL_THREAD_WORKAROUND 1
#endif

namespace WebCore {
    class ArchiveResource;
}

@interface WebResource (WebResourceInternal)
- (id)_initWithCoreResource:(PassRefPtr<WebCore::ArchiveResource>)coreResource;
- (WebCore::ArchiveResource*)_coreResource;
@end

#ifdef MAIL_THREAD_WORKAROUND

@interface WebResource (WebMailThreadWorkaround)
+ (BOOL)_needMailThreadWorkaroundIfCalledOffMainThread;
@end

inline bool needMailThreadWorkaround()
{
    return !pthread_main_np() && [WebResource _needMailThreadWorkaroundIfCalledOffMainThread];
}

#endif
