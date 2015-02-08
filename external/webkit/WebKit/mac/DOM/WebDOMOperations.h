

#import <WebKit/DOMCore.h>
#import <WebKit/DOMHTML.h>
#import <WebKit/DOMRange.h>

@class WebArchive;
@class WebFrame;

@interface DOMNode (WebDOMNodeOperations)

- (WebArchive *)webArchive;

@end

@interface DOMDocument (WebDOMDocumentOperations)

- (WebFrame *)webFrame;

- (NSURL *)URLWithAttributeString:(NSString *)string;

@end

@interface DOMRange (WebDOMRangeOperations)

- (WebArchive *)webArchive;

- (NSString *)markupString;

@end

@interface DOMHTMLFrameElement (WebDOMHTMLFrameElementOperations)

- (WebFrame *)contentFrame;

@end

@interface DOMHTMLIFrameElement (WebDOMHTMLIFrameElementOperations)

- (WebFrame *)contentFrame;

@end

@interface DOMHTMLObjectElement (WebDOMHTMLObjectElementOperations)

- (WebFrame *)contentFrame;

@end
