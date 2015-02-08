
 
#import <Cocoa/Cocoa.h>
#import <WebKit/WebKit.h>

@interface EventSendingController : NSObject <DOMEventListener>
{
    BOOL leftMouseButtonDown;
    BOOL dragMode;
    int clickCount;
    NSTimeInterval lastClick;
    int eventNumber;
    double timeOffset;
}

+ (void)saveEvent:(NSInvocation *)event;
+ (void)replaySavedEvents;
+ (void)clearSavedEvents;

- (void)scheduleAsynchronousClick;

- (void)enableDOMUIEventLogging:(WebScriptObject *)node;

- (void)handleEvent:(DOMEvent *)event;

@end

extern NSPoint lastMousePosition;
extern NSPoint lastClickPosition;