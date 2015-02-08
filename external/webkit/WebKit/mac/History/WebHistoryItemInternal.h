

#import "WebBackForwardList.h"
#import "WebHistoryItemPrivate.h"
#import <wtf/Forward.h>

namespace WebCore {
    class HistoryItem;
}

WebCore::HistoryItem* core(WebHistoryItem *item);
WebHistoryItem *kit(WebCore::HistoryItem* item);

extern void WKNotifyHistoryItemChanged(WebCore::HistoryItem*);

@interface WebHistoryItem (WebInternal)

+ (WebHistoryItem *)entryWithURL:(NSURL *)URL;
+ (void)initWindowWatcherIfNecessary;

- (id)initWithURL:(NSURL *)URL target:(NSString *)target parent:(NSString *)parent title:(NSString *)title;
- (id)initWithURLString:(NSString *)URLString title:(NSString *)title displayTitle:(NSString *)displayTitle lastVisitedTimeInterval:(NSTimeInterval)time;
- (id)initFromDictionaryRepresentation:(NSDictionary *)dict;
- (id)initWithWebCoreHistoryItem:(PassRefPtr<WebCore::HistoryItem>)item;

- (void)_mergeAutoCompleteHints:(WebHistoryItem *)otherItem;
- (void)setTitle:(NSString *)title;
- (void)_visitedWithTitle:(NSString *)title increaseVisitCount:(BOOL)increaseVisitCount;
- (void)_recordInitialVisit;

@end

@interface WebBackForwardList (WebInternal)
- (void)_close;
@end
