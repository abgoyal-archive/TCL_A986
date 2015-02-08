

#import <Cocoa/Cocoa.h>

#if MAC_OS_X_VERSION_MAX_ALLOWED <= MAC_OS_X_VERSION_10_4
#define WebNSInteger int
#else
#define WebNSInteger NSInteger
#endif

@class DOMCSSStyleDeclaration;
@class DOMDocument;
@class DOMElement;
@class DOMNode;
@class DOMRange;

@class WebArchive;
@class WebBackForwardList;
@class WebDataSource;
@class WebFrame;
@class WebFrameView;
@class WebHistoryItem;
@class WebPreferences;
@class WebScriptObject;
@class WebViewPrivate;

// Element dictionary keys
extern NSString *WebElementDOMNodeKey;          // DOMNode of the element
extern NSString *WebElementFrameKey;            // WebFrame of the element
extern NSString *WebElementImageAltStringKey;   // NSString of the ALT attribute of the image element
extern NSString *WebElementImageKey;            // NSImage of the image element
extern NSString *WebElementImageRectKey;        // NSValue of an NSRect, the rect of the image element
extern NSString *WebElementImageURLKey;         // NSURL of the image element
extern NSString *WebElementIsSelectedKey;       // NSNumber of BOOL indicating whether the element is selected or not 
extern NSString *WebElementLinkURLKey;          // NSURL of the link if the element is within an anchor
extern NSString *WebElementLinkTargetFrameKey;  // WebFrame of the target of the anchor
extern NSString *WebElementLinkTitleKey;        // NSString of the title of the anchor
extern NSString *WebElementLinkLabelKey;        // NSString of the text within the anchor

extern NSString *WebViewProgressStartedNotification;
extern NSString *WebViewProgressEstimateChangedNotification;
extern NSString *WebViewProgressFinishedNotification;

@interface WebView : NSView
{
@private
    WebViewPrivate *_private;
}

+ (BOOL)canShowMIMEType:(NSString *)MIMEType;


+ (BOOL)canShowMIMETypeAsHTML:(NSString *)MIMEType;

+ (NSArray *)MIMETypesShownAsHTML;

+ (void)setMIMETypesShownAsHTML:(NSArray *)MIMETypes;

+ (NSURL *)URLFromPasteboard:(NSPasteboard *)pasteboard;

+ (NSString *)URLTitleFromPasteboard:(NSPasteboard *)pasteboard;

+ (void)registerURLSchemeAsLocal:(NSString *)scheme;

- (id)initWithFrame:(NSRect)frame frameName:(NSString *)frameName groupName:(NSString *)groupName;

- (void)close;

- (void)setShouldCloseWithWindow:(BOOL)close;

- (BOOL)shouldCloseWithWindow;

- (void)setUIDelegate:(id)delegate;

- (id)UIDelegate;

- (void)setResourceLoadDelegate:(id)delegate;

- (id)resourceLoadDelegate;

- (void)setDownloadDelegate:(id)delegate;

- (id)downloadDelegate;

- (void)setFrameLoadDelegate:(id)delegate;

- (id)frameLoadDelegate;

- (void)setPolicyDelegate:(id)delegate;

- (id)policyDelegate;

- (WebFrame *)mainFrame;

- (WebFrame *)selectedFrame;

- (WebBackForwardList *)backForwardList;

- (void)setMaintainsBackForwardList:(BOOL)flag;

- (BOOL)goBack;

- (BOOL)goForward;

- (BOOL)goToBackForwardItem:(WebHistoryItem *)item;

- (void)setTextSizeMultiplier:(float)multiplier;

- (float)textSizeMultiplier;

- (void)setApplicationNameForUserAgent:(NSString *)applicationName;

- (NSString *)applicationNameForUserAgent;

- (void)setCustomUserAgent:(NSString *)userAgentString;

- (NSString *)customUserAgent;

- (NSString *)userAgentForURL:(NSURL *)URL;


- (BOOL)supportsTextEncoding;

- (void)setCustomTextEncodingName:(NSString *)encodingName;

- (NSString *)customTextEncodingName;

- (void)setMediaStyle:(NSString *)mediaStyle;

- (NSString *)mediaStyle;

- (NSString *)stringByEvaluatingJavaScriptFromString:(NSString *)script;

- (WebScriptObject *)windowScriptObject;

- (void)setPreferences: (WebPreferences *)prefs;

- (WebPreferences *)preferences;

- (void)setPreferencesIdentifier:(NSString *)anIdentifier;

- (NSString *)preferencesIdentifier;


- (void)setHostWindow:(NSWindow *)hostWindow;

- (NSWindow *)hostWindow;

- (BOOL)searchFor:(NSString *)string direction:(BOOL)forward caseSensitive:(BOOL)caseFlag wrap:(BOOL)wrapFlag;

+ (void)registerViewClass:(Class)viewClass representationClass:(Class)representationClass forMIMEType:(NSString *)MIMEType;


- (void)setGroupName:(NSString *)groupName;

- (NSString *)groupName;

- (double)estimatedProgress;

- (BOOL)isLoading;

- (NSDictionary *)elementAtPoint:(NSPoint)point;

- (NSArray *)pasteboardTypesForSelection;

- (void)writeSelectionWithPasteboardTypes:(NSArray *)types toPasteboard:(NSPasteboard *)pasteboard;

- (NSArray *)pasteboardTypesForElement:(NSDictionary *)element;

- (void)writeElement:(NSDictionary *)element withPasteboardTypes:(NSArray *)types toPasteboard:(NSPasteboard *)pasteboard;

- (void)moveDragCaretToPoint:(NSPoint)point;

- (void)removeDragCaret;

- (void)setDrawsBackground:(BOOL)drawsBackround;

- (BOOL)drawsBackground;

- (void)setShouldUpdateWhileOffscreen:(BOOL)updateWhileOffscreen;

- (BOOL)shouldUpdateWhileOffscreen;

- (void)setMainFrameURL:(NSString *)URLString;

- (NSString *)mainFrameURL;

- (DOMDocument *)mainFrameDocument;

- (NSString *)mainFrameTitle;

- (NSImage *)mainFrameIcon;

@end


@interface WebView (WebIBActions) <NSUserInterfaceValidations>
- (IBAction)takeStringURLFrom:(id)sender;
- (IBAction)stopLoading:(id)sender;
- (IBAction)reload:(id)sender;
- (IBAction)reloadFromOrigin:(id)sender;
- (BOOL)canGoBack;
- (IBAction)goBack:(id)sender;
- (BOOL)canGoForward;
- (IBAction)goForward:(id)sender;
- (BOOL)canMakeTextLarger;
- (IBAction)makeTextLarger:(id)sender;
- (BOOL)canMakeTextSmaller;
- (IBAction)makeTextSmaller:(id)sender;
- (BOOL)canMakeTextStandardSize;
- (IBAction)makeTextStandardSize:(id)sender;
- (IBAction)toggleContinuousSpellChecking:(id)sender;
- (IBAction)toggleSmartInsertDelete:(id)sender;
@end


// WebView editing support

extern NSString * const WebViewDidBeginEditingNotification;
extern NSString * const WebViewDidChangeNotification;
extern NSString * const WebViewDidEndEditingNotification;
extern NSString * const WebViewDidChangeTypingStyleNotification;
extern NSString * const WebViewDidChangeSelectionNotification;

@interface WebView (WebViewCSS)
- (DOMCSSStyleDeclaration *)computedStyleForElement:(DOMElement *)element pseudoElement:(NSString *)pseudoElement;
@end

@interface WebView (WebViewEditing)
- (DOMRange *)editableDOMRangeForPoint:(NSPoint)point;
- (void)setSelectedDOMRange:(DOMRange *)range affinity:(NSSelectionAffinity)selectionAffinity;
- (DOMRange *)selectedDOMRange;
- (NSSelectionAffinity)selectionAffinity;
- (BOOL)maintainsInactiveSelection;
- (void)setEditable:(BOOL)flag;
- (BOOL)isEditable;
- (void)setTypingStyle:(DOMCSSStyleDeclaration *)style;
- (DOMCSSStyleDeclaration *)typingStyle;
- (void)setSmartInsertDeleteEnabled:(BOOL)flag;
- (BOOL)smartInsertDeleteEnabled;
- (void)setContinuousSpellCheckingEnabled:(BOOL)flag;
- (BOOL)isContinuousSpellCheckingEnabled;
- (WebNSInteger)spellCheckerDocumentTag;
- (NSUndoManager *)undoManager;
- (void)setEditingDelegate:(id)delegate;
- (id)editingDelegate;
- (DOMCSSStyleDeclaration *)styleDeclarationWithText:(NSString *)text;
@end

@interface WebView (WebViewUndoableEditing)
- (void)replaceSelectionWithNode:(DOMNode *)node; 
- (void)replaceSelectionWithText:(NSString *)text;    
- (void)replaceSelectionWithMarkupString:(NSString *)markupString;
- (void)replaceSelectionWithArchive:(WebArchive *)archive;
- (void)deleteSelection;    
- (void)applyStyle:(DOMCSSStyleDeclaration *)style;
@end

@interface WebView (WebViewEditingActions)

- (void)copy:(id)sender;
- (void)cut:(id)sender;
- (void)paste:(id)sender;
- (void)copyFont:(id)sender;
- (void)pasteFont:(id)sender;
- (void)delete:(id)sender;
- (void)pasteAsPlainText:(id)sender;
- (void)pasteAsRichText:(id)sender;

- (void)changeFont:(id)sender;
- (void)changeAttributes:(id)sender;
- (void)changeDocumentBackgroundColor:(id)sender;
- (void)changeColor:(id)sender;

- (void)alignCenter:(id)sender;
- (void)alignJustified:(id)sender;
- (void)alignLeft:(id)sender;
- (void)alignRight:(id)sender;

- (void)checkSpelling:(id)sender;
- (void)showGuessPanel:(id)sender;
- (void)performFindPanelAction:(id)sender;

- (void)startSpeaking:(id)sender;
- (void)stopSpeaking:(id)sender;

- (void)moveToBeginningOfSentence:(id)sender;
- (void)moveToBeginningOfSentenceAndModifySelection:(id)sender;
- (void)moveToEndOfSentence:(id)sender;
- (void)moveToEndOfSentenceAndModifySelection:(id)sender;
- (void)selectSentence:(id)sender;

 
@end

#undef WebNSInteger
