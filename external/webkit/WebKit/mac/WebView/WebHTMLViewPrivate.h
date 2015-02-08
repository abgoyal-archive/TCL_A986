

#import <WebKit/WebHTMLView.h>

#if !defined(ENABLE_NETSCAPE_PLUGIN_API)
#define ENABLE_NETSCAPE_PLUGIN_API 1
#endif

@class DOMDocumentFragment;
@class DOMNode;
@class DOMRange;
@class WebPluginController;

@protocol WebHTMLHighlighter
- (NSRect)highlightRectForLine:(NSRect)lineRect representedNode:(DOMNode *)node;
- (void)paintHighlightForBox:(NSRect)boxRect onLine:(NSRect)lineRect behindText:(BOOL)text entireLine:(BOOL)line representedNode:(DOMNode *)node;

// the following methods are deprecated and will be removed once Mail switches to the new methods <rdar://problem/5050528>
- (NSRect)highlightRectForLine:(NSRect)lineRect;
- (void)paintHighlightForBox:(NSRect)boxRect onLine:(NSRect)lineRect behindText:(BOOL)text entireLine:(BOOL)line;
@end

@interface WebHTMLView (WebPrivate)

+ (NSArray *)supportedMIMETypes;
+ (NSArray *)supportedImageMIMETypes;
+ (NSArray *)supportedNonImageMIMETypes;
+ (NSArray *)unsupportedTextMIMETypes;

- (void)close;

// Modifier (flagsChanged) tracking SPI
+ (void)_postFlagsChangedEvent:(NSEvent *)flagsChangedEvent;
- (void)_updateMouseoverWithFakeEvent;

- (void)_setAsideSubviews;
- (void)_restoreSubviews;

- (BOOL)_insideAnotherHTMLView;
- (void)_clearLastHitViewIfSelf;
- (void)_updateMouseoverWithEvent:(NSEvent *)event;

+ (NSArray *)_insertablePasteboardTypes;
+ (NSArray *)_selectionPasteboardTypes;
- (void)_writeSelectionToPasteboard:(NSPasteboard *)pasteboard;

- (void)_frameOrBoundsChanged;

- (NSImage *)_dragImageForLinkElement:(NSDictionary *)element;
- (NSImage *)_dragImageForURL:(NSString*)linkURL withLabel:(NSString*)label;
- (void)_handleAutoscrollForMouseDragged:(NSEvent *)event;
- (WebPluginController *)_pluginController;

// FIXME: _selectionRect is deprecated in favor of selectionRect, which is in protocol WebDocumentSelection.
// We can't remove this yet because it's still in use by Mail.
- (NSRect)_selectionRect;

- (void)_startAutoscrollTimer:(NSEvent *)event;
- (void)_stopAutoscrollTimer;

- (BOOL)_canEdit;
- (BOOL)_canEditRichly;
- (BOOL)_canAlterCurrentSelection;
- (BOOL)_hasSelection;
- (BOOL)_hasSelectionOrInsertionPoint;
- (BOOL)_isEditable;

- (BOOL)_transparentBackground;
- (void)_setTransparentBackground:(BOOL)isBackgroundTransparent;

- (void)_setToolTip:(NSString *)string;

// SPI used by Mail.
// FIXME: These should all be moved to WebView; we won't always have a WebHTMLView.
- (NSImage *)_selectionDraggingImage;
- (NSRect)_selectionDraggingRect;
- (DOMNode *)_insertOrderedList;
- (DOMNode *)_insertUnorderedList;
- (BOOL)_canIncreaseSelectionListLevel;
- (BOOL)_canDecreaseSelectionListLevel;
- (DOMNode *)_increaseSelectionListLevel;
- (DOMNode *)_increaseSelectionListLevelOrdered;
- (DOMNode *)_increaseSelectionListLevelUnordered;
- (void)_decreaseSelectionListLevel;
- (void)_setHighlighter:(id <WebHTMLHighlighter>)highlighter ofType:(NSString *)type;
- (void)_removeHighlighterOfType:(NSString *)type;
- (DOMDocumentFragment *)_documentFragmentFromPasteboard:(NSPasteboard *)pasteboard forType:(NSString *)pboardType inContext:(DOMRange *)context subresources:(NSArray **)subresources;

#if ENABLE_NETSCAPE_PLUGIN_API
- (void)_resumeNullEventsForAllNetscapePlugins;
- (void)_pauseNullEventsForAllNetscapePlugins;
#endif

- (BOOL)_isUsingAcceleratedCompositing;
- (NSView *)_compositingLayersHostingView;

// SPI for printing (should be converted to API someday). When the WebHTMLView isn't being printed
// directly, this method must be called before paginating, or the computed height might be incorrect.
// Typically this would be called from inside an override of -[NSView knowsPageRange:].
- (void)_layoutForPrinting;

- (BOOL)_canSmartReplaceWithPasteboard:(NSPasteboard *)pasteboard;

@end
