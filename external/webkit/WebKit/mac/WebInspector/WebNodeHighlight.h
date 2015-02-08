

@class WebNodeHighlightView;

namespace WebCore {
    class InspectorController;
}

@interface WebNodeHighlight : NSObject {
    NSView *_targetView;
    NSWindow *_highlightWindow;
    WebNodeHighlightView *_highlightView;
    WebCore::InspectorController* _inspectorController;
    id _delegate;
}
- (id)initWithTargetView:(NSView *)targetView inspectorController:(WebCore::InspectorController*)inspectorController;

- (void)setDelegate:(id)delegate;
- (id)delegate;

- (void)attach;
- (void)detach;

- (NSView *)targetView;
- (WebNodeHighlightView *)highlightView;

- (WebCore::InspectorController*)inspectorController;

- (void)setNeedsUpdateInTargetViewRect:(NSRect)rect;
@end

@interface NSObject (WebNodeHighlightDelegate)
- (void)didAttachWebNodeHighlight:(WebNodeHighlight *)highlight;
- (void)willDetachWebNodeHighlight:(WebNodeHighlight *)highlight;
@end
