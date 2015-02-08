

#import <WebKit/WebIconFetcher.h>
#import <wtf/Forward.h>

namespace WebCore {
    class IconFetcher;
}

@class WebFrame;

@interface WebIconFetcher (WebInternal)

+ (WebIconFetcher *)_fetchApplicationIconForFrame:(WebFrame *)webFrame
                                           target:(id)target
                                         selector:(SEL)selector;

@end

