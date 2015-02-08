

@class WebNodeHighlight;

@interface WebNodeHighlightView : NSView {
    WebNodeHighlight *_webNodeHighlight;
}
- (id)initWithWebNodeHighlight:(WebNodeHighlight *)webNodeHighlight;

- (WebNodeHighlight *)webNodeHighlight;
- (void)detachFromWebNodeHighlight;
@end
