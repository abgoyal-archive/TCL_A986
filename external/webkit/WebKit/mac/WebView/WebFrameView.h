

#import <Cocoa/Cocoa.h>

@class WebDataSource;
@class WebFrame;
@class WebFrameViewPrivate;

@protocol WebDocumentView;

@interface WebFrameView : NSView
{
@private
    WebFrameViewPrivate *_private;
}

- (WebFrame *)webFrame;

- (NSView <WebDocumentView> *)documentView;

- (void)setAllowsScrolling:(BOOL)flag;

- (BOOL)allowsScrolling;

- (BOOL)canPrintHeadersAndFooters;

- (NSPrintOperation *)printOperationWithPrintInfo:(NSPrintInfo *)printInfo;

- (BOOL)documentViewShouldHandlePrint;

- (void)printDocumentView;

@end
