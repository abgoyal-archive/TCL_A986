

#import <WebCore/DOMException.h>
#import <WebCore/WebScriptObject.h>

#if WEBKIT_VERSION_MAX_ALLOWED >= WEBKIT_VERSION_1_3

@class DOMStyleSheet;

typedef unsigned long long DOMTimeStamp;

typedef struct DOMObjectInternal DOMObjectInternal;

@interface DOMObject : WebScriptObject <NSCopying>
{
@private
    DOMObjectInternal *_internal;
}
@end

@interface DOMObject (DOMLinkStyle)
#ifndef BUILDING_ON_TIGER
@property(readonly, retain) DOMStyleSheet *sheet;
#else
- (DOMStyleSheet *)sheet;
#endif
@end

#endif
