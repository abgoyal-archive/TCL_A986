

#import <WebKit/WebDocument.h>
#import <WebKit/WebHTMLView.h>

@class DOMDocument;

@protocol WebDocumentImage <NSObject>
- (NSImage *)image;
@end

// This method is deprecated as it now lives on WebFrame.
@protocol WebDocumentDOM <NSObject>
- (DOMDocument *)DOMDocument;
- (BOOL)canSaveAsWebArchive;
@end

@protocol WebDocumentSelection <WebDocumentText>
- (NSArray *)pasteboardTypesForSelection;
- (void)writeSelectionWithPasteboardTypes:(NSArray *)types toPasteboard:(NSPasteboard *)pasteboard;

// Array of rects that tightly enclose the selected text, in coordinates of selectinView.
- (NSArray *)selectionTextRects;

// Rect tightly enclosing the entire selected area, in coordinates of selectionView.
- (NSRect)selectionRect;

// NSImage of the portion of the selection that's in view. This does not draw backgrounds. 
// The text is all black according to the parameter.
- (NSImage *)selectionImageForcingBlackText:(BOOL)forceBlackText;

// Rect tightly enclosing the entire selected area, in coordinates of selectionView.
// NOTE: This method is equivalent to selectionRect and shouldn't be used; use selectionRect instead.
- (NSRect)selectionImageRect;

// View that draws the selection and can be made first responder. Often this is self but it could be
// a nested view, as for example in the case of WebPDFView.
- (NSView *)selectionView;
@end

@protocol WebDocumentIncrementalSearching
- (BOOL)searchFor:(NSString *)string direction:(BOOL)forward caseSensitive:(BOOL)caseFlag wrap:(BOOL)wrapFlag startInSelection:(BOOL)startInSelection;
@end

@interface WebHTMLView (WebDocumentPrivateProtocols) <WebDocumentSelection, WebDocumentIncrementalSearching>
@end
