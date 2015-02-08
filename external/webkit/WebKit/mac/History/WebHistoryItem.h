

#import <Cocoa/Cocoa.h>

@class WebHistoryItemPrivate;
@class NSURL;

extern NSString *WebHistoryItemChangedNotification;

@interface WebHistoryItem : NSObject <NSCopying>
{
@private
    WebHistoryItemPrivate *_private;
}

- (id)initWithURLString:(NSString *)URLString title:(NSString *)title lastVisitedTimeInterval:(NSTimeInterval)time;

- (NSString *)originalURLString;

- (NSString *)URLString;


- (NSString *)title;

- (NSTimeInterval)lastVisitedTimeInterval;

- (void)setAlternateTitle:(NSString *)alternateTitle;

- (NSString *)alternateTitle;

- (NSImage *)icon;

@end
