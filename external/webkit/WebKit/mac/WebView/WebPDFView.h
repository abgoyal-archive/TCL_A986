

#import <WebKit/WebDocumentInternal.h>

@class PDFDocument;
@class PDFView;
@class WebDataSource;

@interface WebPDFView : NSView <WebDocumentView, WebDocumentSearching, WebDocumentIncrementalSearching, WebMultipleTextMatches, WebDocumentSelection, WebDocumentElement, _WebDocumentViewState, _WebDocumentZooming>
{
    NSView *previewView;
    PDFView *PDFSubview;
    NSString *path;
    BOOL firstResponderIsPDFDocumentView;
    BOOL written;
    BOOL _ignoreScaleAndDisplayModeAndPageNotifications;
    BOOL _willUpdatePreferencesSoon;
    PDFView *PDFSubviewProxy;
    WebDataSource *dataSource;
    NSArray *textMatches;
    NSPoint lastScrollPosition;
}

+ (NSArray *)supportedMIMETypes;
+ (NSBundle *)PDFKitBundle;

- (void)setPDFDocument:(PDFDocument *)doc;

@end
