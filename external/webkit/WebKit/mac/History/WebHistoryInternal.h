

#import "WebHistoryPrivate.h"

namespace WebCore {
    class PageGroup;
}

@interface WebHistory (WebInternal)
- (void)_visitedURL:(NSURL *)URL withTitle:(NSString *)title method:(NSString *)method wasFailure:(BOOL)wasFailure increaseVisitCount:(BOOL)increaseVisitCount;
- (void)_addVisitedLinksToPageGroup:(WebCore::PageGroup&)group;
@end
