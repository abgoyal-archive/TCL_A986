

#import <Foundation/Foundation.h>

#import <WebKit/WebDocumentPrivate.h>

#if MAC_OS_X_VERSION_MAX_ALLOWED <= MAC_OS_X_VERSION_10_4
#define WebNSUInteger unsigned int
#else
#define WebNSUInteger NSUInteger
#endif

@class WebHTMLRepresentationPrivate;
@class NSView;

@class DOMNode;
@class DOMElement;

@protocol WebDocumentMarkup;
@protocol WebDocumentRepresentation;
@protocol WebDocumentSourceRepresentation;

@interface WebHTMLRepresentation : NSObject <WebDocumentRepresentation, WebDocumentDOM>
{
    WebHTMLRepresentationPrivate *_private;
}

+ (NSArray *)supportedMIMETypes;
+ (NSArray *)supportedNonImageMIMETypes;
+ (NSArray *)supportedImageMIMETypes;

- (NSAttributedString *)attributedStringFrom:(DOMNode *)startNode startOffset:(int)startOffset to:(DOMNode *)endNode endOffset:(int)endOffset;

- (DOMElement *)elementWithName:(NSString *)name inForm:(DOMElement *)form;
- (BOOL)elementDoesAutoComplete:(DOMElement *)element;
- (BOOL)elementIsPassword:(DOMElement *)element;
- (DOMElement *)formForElement:(DOMElement *)element;
- (DOMElement *)currentForm;
- (NSArray *)controlsInForm:(DOMElement *)form;
- (NSString *)searchForLabels:(NSArray *)labels beforeElement:(DOMElement *)element resultDistance:(WebNSUInteger*)outDistance resultIsInCellAbove:(BOOL*)outIsInCellAbove;
- (NSString *)matchLabels:(NSArray *)labels againstElement:(DOMElement *)element;

// Deprecated SPI
- (NSString *)searchForLabels:(NSArray *)labels beforeElement:(DOMElement *)element; // Use -searchForLabels:beforeElement:resultDistance:resultIsInCellAbove:

@end

#undef WebNSUInteger
