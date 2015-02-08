

#import <Foundation/NSDictionary.h>

namespace WebCore { 
    class HitTestResult;
}

@interface WebElementDictionary : NSDictionary {
    WebCore::HitTestResult* _result;
    NSMutableDictionary *_cache;
    NSMutableSet *_nilValues;
    BOOL _cacheComplete;
}
- (id)initWithHitTestResult:(const WebCore::HitTestResult&)result;
@end
