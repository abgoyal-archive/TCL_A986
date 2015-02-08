

#include "WebSecurityOriginPrivate.h"

namespace WebCore {
    class SecurityOrigin;
}

typedef WebCore::SecurityOrigin WebCoreSecurityOrigin;

@interface WebSecurityOrigin (WebInternal)

- (id)_initWithWebCoreSecurityOrigin:(WebCoreSecurityOrigin *)origin;
- (WebCoreSecurityOrigin *)_core;

@end
