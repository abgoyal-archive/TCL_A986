

#import <WebKit/WebDOMOperations.h>

@interface DOMDocument (WebDOMDocumentOperationsPrivate)
- (NSArray *)_focusableNodes;
@end

@interface DOMNode (WebDOMNodeOperationsPendingPublic)
- (NSString *)markupString;
@end
