

#import <WebCore/DOMAttr.h>
#import <WebCore/DOMCSS.h>
#import <WebCore/DOMCSSStyleDeclaration.h>
#import <WebCore/DOMDocument.h>
#import <WebCore/DOMElement.h>
#import <WebCore/DOMHTML.h>
#import <WebCore/DOMHTMLAnchorElement.h>
#import <WebCore/DOMHTMLAreaElement.h>
#import <WebCore/DOMHTMLDocument.h>
#import <WebCore/DOMHTMLElement.h>
#import <WebCore/DOMHTMLEmbedElement.h>
#import <WebCore/DOMHTMLImageElement.h>
#import <WebCore/DOMHTMLInputElement.h>
#import <WebCore/DOMHTMLLinkElement.h>
#import <WebCore/DOMHTMLObjectElement.h>
#import <WebCore/DOMNode.h>
#import <WebCore/DOMRGBColor.h>
#import <WebCore/DOMRange.h>

@class NSArray;
@class NSImage;
@class NSURL;

@interface DOMNode (DOMNodeExtensions)
- (NSRect)boundingBox WEBKIT_OBJC_METHOD_ANNOTATION(AVAILABLE_WEBKIT_VERSION_3_0_AND_LATER);
- (NSArray *)lineBoxRects WEBKIT_OBJC_METHOD_ANNOTATION(AVAILABLE_WEBKIT_VERSION_3_0_AND_LATER);
@end

@interface DOMElement (DOMElementAppKitExtensions)
- (NSImage *)image WEBKIT_OBJC_METHOD_ANNOTATION(AVAILABLE_WEBKIT_VERSION_3_0_AND_LATER);
@end

@interface DOMHTMLDocument (DOMHTMLDocumentExtensions)
- (DOMDocumentFragment *)createDocumentFragmentWithMarkupString:(NSString *)markupString baseURL:(NSURL *)baseURL WEBKIT_OBJC_METHOD_ANNOTATION(AVAILABLE_WEBKIT_VERSION_3_0_AND_LATER);
- (DOMDocumentFragment *)createDocumentFragmentWithText:(NSString *)text WEBKIT_OBJC_METHOD_ANNOTATION(AVAILABLE_WEBKIT_VERSION_3_0_AND_LATER);
@end
