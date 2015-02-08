

#import <Foundation/Foundation.h>

#if MAC_OS_X_VERSION_MAX_ALLOWED <= MAC_OS_X_VERSION_10_4
#define WebNSUInteger unsigned int
#else
#define WebNSUInteger NSUInteger
#endif

@class DOMRange;
@class DOMNode;
@class WebTextIteratorPrivate;

@interface WebTextIterator : NSObject {
@private
    WebTextIteratorPrivate *_private;
}

- (id)initWithRange:(DOMRange *)range;

- (void)advance;

- (BOOL)atEnd;

- (WebNSUInteger)currentTextLength;

- (const unichar *)currentTextPointer;

- (DOMRange *)currentRange;

@end

@interface WebTextIterator (WebTextIteratorDeprecated)

- (DOMNode *)currentNode;

- (NSString *)currentText;

@end

#undef WebNSUInteger
