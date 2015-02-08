

#import <WebKit/WebBackForwardList.h>
#import <wtf/PassRefPtr.h>

namespace WebCore {
    class BackForwardList;
}

WebCore::BackForwardList* core(WebBackForwardList *);
WebBackForwardList *kit(WebCore::BackForwardList*);

@interface WebBackForwardList (WebBackForwardListInternal)
- (id)initWithBackForwardList:(PassRefPtr<WebCore::BackForwardList>)backForwardList;
@end
