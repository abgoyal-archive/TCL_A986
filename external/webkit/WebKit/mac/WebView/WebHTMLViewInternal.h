

// Things internal to the WebKit framework; not SPI.

#import "WebHTMLViewPrivate.h"

#if USE(ACCELERATED_COMPOSITING)
@class CALayer;
#endif
@class WebFrame;

namespace WebCore {
    class CachedImage;
    class KeyboardEvent;
}

@interface WebHTMLView (WebInternal)
- (void)_selectionChanged;
- (void)_updateFontPanel;
- (BOOL)_canSmartCopyOrDelete;

- (id <WebHTMLHighlighter>)_highlighterForType:(NSString*)type;
- (WebFrame *)_frame;
- (void)_lookUpInDictionaryFromMenu:(id)sender;
- (void)_hoverFeedbackSuspendedChanged;
- (BOOL)_interceptEditingKeyEvent:(WebCore::KeyboardEvent *)event shouldSaveCommand:(BOOL)shouldSave;
- (DOMDocumentFragment *)_documentFragmentFromPasteboard:(NSPasteboard *)pasteboard;
- (NSEvent *)_mouseDownEvent;
#ifndef BUILDING_ON_TIGER
- (BOOL)isGrammarCheckingEnabled;
- (void)setGrammarCheckingEnabled:(BOOL)flag;
- (void)toggleGrammarChecking:(id)sender;
#endif
- (WebCore::CachedImage*)promisedDragTIFFDataSource;
- (void)setPromisedDragTIFFDataSource:(WebCore::CachedImage*)source;
- (void)_web_layoutIfNeededRecursive;
- (void)_destroyAllWebPlugins;
- (BOOL)_needsLayout;

#if USE(ACCELERATED_COMPOSITING)
- (void)attachRootLayer:(CALayer*)layer;
- (void)detachRootLayer;
#endif

#if USE(ACCELERATED_COMPOSITING) && defined(BUILDING_ON_LEOPARD)
- (void)_updateLayerHostingViewPosition;
#endif

@end
