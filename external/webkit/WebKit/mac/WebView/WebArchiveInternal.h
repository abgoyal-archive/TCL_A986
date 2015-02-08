

#import "WebArchive.h"
#import <JavaScriptCore/Forward.h>

namespace WebCore {
    class LegacyWebArchive;
}

@interface WebArchive (WebInternal)

- (id)_initWithCoreLegacyWebArchive:(WTF::PassRefPtr<WebCore::LegacyWebArchive>)coreLegacyWebArchive;
- (WebCore::LegacyWebArchive *)_coreLegacyWebArchive;

@end
