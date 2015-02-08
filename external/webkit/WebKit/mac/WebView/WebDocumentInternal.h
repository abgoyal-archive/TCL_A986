

#import <WebKit/WebDocumentPrivate.h>
#import <WebKit/WebHTMLView.h>

#if MAC_OS_X_VERSION_MAX_ALLOWED <= MAC_OS_X_VERSION_10_4
#define WebNSUInteger unsigned int
#else
#define WebNSUInteger NSUInteger
#endif

@protocol _WebDocumentZooming <NSObject>

// Methods to perform the actual commands
- (IBAction)_zoomOut:(id)sender;
- (IBAction)_zoomIn:(id)sender;
- (IBAction)_resetZoom:(id)sender;

// Whether or not the commands can be executed.
- (BOOL)_canZoomOut;
- (BOOL)_canZoomIn;
- (BOOL)_canResetZoom;

@end

@protocol WebDocumentElement <NSObject>
- (NSDictionary *)elementAtPoint:(NSPoint)point;
- (NSDictionary *)elementAtPoint:(NSPoint)point allowShadowContent:(BOOL)allow;
@end

@protocol WebMultipleTextMatches <NSObject>
- (void)setMarkedTextMatchesAreHighlighted:(BOOL)newValue;
- (BOOL)markedTextMatchesAreHighlighted;
- (WebNSUInteger)markAllMatchesForText:(NSString *)string caseSensitive:(BOOL)caseFlag limit:(WebNSUInteger)limit;
- (void)unmarkAllTextMatches;
- (NSArray *)rectsForTextMatches;
@end


/* Used to save and restore state in the view, typically when going back/forward */
@protocol _WebDocumentViewState <NSObject>
- (NSPoint)scrollPoint;
- (void)setScrollPoint:(NSPoint)p;
- (id)viewState;
- (void)setViewState:(id)statePList;
@end

@interface WebHTMLView (WebDocumentInternalProtocols) <WebDocumentElement, WebMultipleTextMatches>
@end

#undef WebNSUInteger
