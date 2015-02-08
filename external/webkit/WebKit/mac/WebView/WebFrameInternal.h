

// This header contains WebFrame declarations that can be used anywhere in WebKit, but are neither SPI nor API.

#import "WebFramePrivate.h"
#import "WebPreferencesPrivate.h"

#import <WebCore/EditAction.h>
#import <WebCore/FrameLoaderTypes.h>
#import <WebCore/SelectionController.h>
#import <WebCore/Settings.h>

@class DOMCSSStyleDeclaration;
@class DOMDocumentFragment;
@class DOMElement;
@class DOMNode;
@class DOMRange;
@class WebFrameView;
@class WebHistoryItem;

class WebScriptDebugger;

namespace WebCore {
    class CSSStyleDeclaration;
    class Document;
    class DocumentLoader;
    class Element;
    class Frame;
    class Frame;
    class HistoryItem;
    class HTMLElement;
    class HTMLFrameOwnerElement;
    class Node;
    class Page;
    class Range;
}

typedef WebCore::HistoryItem WebCoreHistoryItem;

WebCore::Frame* core(WebFrame *);
WebFrame *kit(WebCore::Frame *);

WebCore::Page* core(WebView *);
WebView *kit(WebCore::Page*);

WebCore::EditableLinkBehavior core(WebKitEditableLinkBehavior);
WebCore::TextDirectionSubmenuInclusionBehavior core(WebTextDirectionSubmenuInclusionBehavior);

WebView *getWebView(WebFrame *webFrame);

@interface WebFramePrivate : NSObject {
@public
    WebCore::Frame* coreFrame;
    WebFrameView *webFrameView;
    WebScriptDebugger* scriptDebugger;
    id internalLoadDelegate;
    BOOL shouldCreateRenderers;
}
@end

@protocol WebCoreRenderTreeCopier <NSObject>
- (NSObject *)nodeWithName:(NSString *)name position:(NSPoint)position rect:(NSRect)rect view:(NSView *)view children:(NSArray *)children;
@end

@interface WebFrame (WebInternal)

+ (void)_createMainFrameWithPage:(WebCore::Page*)page frameName:(const WebCore::String&)name frameView:(WebFrameView *)frameView;
+ (PassRefPtr<WebCore::Frame>)_createSubframeWithOwnerElement:(WebCore::HTMLFrameOwnerElement*)ownerElement frameName:(const WebCore::String&)name frameView:(WebFrameView *)frameView;
- (id)_initWithWebFrameView:(WebFrameView *)webFrameView webView:(WebView *)webView;

- (void)_clearCoreFrame;

- (void)_updateBackgroundAndUpdatesWhileOffscreen;
- (void)_setInternalLoadDelegate:(id)internalLoadDelegate;
- (id)_internalLoadDelegate;
#ifndef BUILDING_ON_TIGER
- (void)_unmarkAllBadGrammar;
#endif
- (void)_unmarkAllMisspellings;

- (BOOL)_hasSelection;
- (void)_clearSelection;
- (WebFrame *)_findFrameWithSelection;
- (void)_clearSelectionInOtherFrames;

- (void)_attachScriptDebugger;
- (void)_detachScriptDebugger;

// dataSource reports null for the initial empty document's data source; this is needed
// to preserve compatibility with Mail and Safari among others. But internal to WebKit,
// we need to be able to get the initial data source as well, so the _dataSource method
// should be used instead.
- (WebDataSource *)_dataSource;

- (BOOL)_needsLayout;
- (void)_drawRect:(NSRect)rect contentsOnly:(BOOL)contentsOnly;
- (BOOL)_getVisibleRect:(NSRect*)rect;
- (NSArray*)_computePageRectsWithPrintWidthScaleFactor:(float)printWidthScaleFactor printHeight:(float)printHeight;

- (NSString *)_stringByEvaluatingJavaScriptFromString:(NSString *)string;
- (NSString *)_stringByEvaluatingJavaScriptFromString:(NSString *)string forceUserGesture:(BOOL)forceUserGesture;

- (NSString *)_selectedString;
- (NSString *)_stringForRange:(DOMRange *)range;

- (NSString *)_markupStringFromRange:(DOMRange *)range nodes:(NSArray **)nodes;

- (NSRect)_caretRectAtNode:(DOMNode *)node offset:(int)offset affinity:(NSSelectionAffinity)affinity;
- (NSRect)_firstRectForDOMRange:(DOMRange *)range;
- (void)_scrollDOMRangeToVisible:(DOMRange *)range;

- (id)_accessibilityTree;

- (DOMRange *)_rangeByAlteringCurrentSelection:(WebCore::SelectionController::EAlteration)alteration direction:(WebCore::SelectionController::EDirection)direction granularity:(WebCore::TextGranularity)granularity;
- (void)_smartInsertForString:(NSString *)pasteString replacingRange:(DOMRange *)charRangeToReplace beforeString:(NSString **)beforeString afterString:(NSString **)afterString;
- (NSRange)_convertToNSRange:(WebCore::Range*)range;
- (DOMRange *)_convertNSRangeToDOMRange:(NSRange)range;
- (NSRange)_convertDOMRangeToNSRange:(DOMRange *)range;

- (DOMDocumentFragment *)_documentFragmentWithMarkupString:(NSString *)markupString baseURLString:(NSString *)baseURLString;
- (DOMDocumentFragment *)_documentFragmentWithNodesAsParagraphs:(NSArray *)nodes;

- (void)_replaceSelectionWithNode:(DOMNode *)node selectReplacement:(BOOL)selectReplacement smartReplace:(BOOL)smartReplace matchStyle:(BOOL)matchStyle;

- (void)_insertParagraphSeparatorInQuotedContent;

- (DOMRange *)_characterRangeAtPoint:(NSPoint)point;

- (DOMCSSStyleDeclaration *)_typingStyle;
- (void)_setTypingStyle:(DOMCSSStyleDeclaration *)style withUndoAction:(WebCore::EditAction)undoAction;

- (void)_dragSourceEndedAt:(NSPoint)windowLoc operation:(NSDragOperation)operation;

- (BOOL)_canProvideDocumentSource;
- (BOOL)_canSaveAsWebArchive;

- (void)_receivedData:(NSData *)data textEncodingName:(NSString *)textEncodingName;

@end

@interface NSObject (WebInternalFrameLoadDelegate)
- (void)webFrame:(WebFrame *)webFrame didFinishLoadWithError:(NSError *)error;
@end
