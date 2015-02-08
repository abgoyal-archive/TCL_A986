

#import "WebScriptWorld.h"

namespace WebCore {
    class DOMWrapperWorld;
}

WebCore::DOMWrapperWorld* core(WebScriptWorld *);

@interface WebScriptWorld (WebInternal)
+ (WebScriptWorld *)findOrCreateWorld:(WebCore::DOMWrapperWorld*)world;
@end
