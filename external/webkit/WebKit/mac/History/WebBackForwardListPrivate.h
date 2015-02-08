

#import <Cocoa/Cocoa.h>

#import <WebKit/WebBackForwardList.h>

@interface WebBackForwardList (WebBackForwardListPrivate)

- (void)removeItem:(WebHistoryItem *)item;

@end
