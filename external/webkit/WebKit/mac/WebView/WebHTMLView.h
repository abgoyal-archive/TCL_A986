

#import <WebKit/WebDocument.h>

@class WebDataSource;
@class WebHTMLViewPrivate;

@interface WebHTMLView : NSControl <WebDocumentView, WebDocumentSearching>
{
@private
    WebHTMLViewPrivate *_private;
}

- (void)setNeedsToApplyStyles:(BOOL)flag;

- (void)reapplyStyles;

- (void)outdent:(id)sender;

@end

