

@class WebView;
@class WebHTMLView;

@interface WebTextCompletionController : NSObject <NSTableViewDelegate, NSTableViewDataSource> {
@private
    WebView *_view;
    WebHTMLView *_htmlView;
    NSWindow *_popupWindow;
    NSTableView *_tableView;
    NSArray *_completions;
    NSString *_originalString;
    int prefixLength;
}
- (id)initWithWebView:(WebView *)view HTMLView:(WebHTMLView *)htmlView;
- (void)doCompletion;
- (void)endRevertingChange:(BOOL)revertChange moveLeft:(BOOL)goLeft;
- (BOOL)popupWindowIsOpen;
- (BOOL)filterKeyDown:(NSEvent *)event;
- (void)_reflectSelection;
@end
