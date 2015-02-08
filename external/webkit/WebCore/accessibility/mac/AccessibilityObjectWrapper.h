

#ifndef AccessibilityObjectWrapper_h
#define AccessibilityObjectWrapper_h

#include <wtf/RefPtr.h>

#ifdef __OBJC__
@class WebCoreTextMarker;
@class WebCoreTextMarkerRange;
#else
class WebCoreTextMarker;
class WebCoreTextMarkerRange;
#endif

namespace WebCore {
class AccessibilityObject;
class VisiblePosition;
}

@interface AccessibilityObjectWrapper : NSObject {
    WebCore::AccessibilityObject* m_object;
}
 
- (id)initWithAccessibilityObject:(WebCore::AccessibilityObject*)axObject;
- (void)detach;
- (WebCore::AccessibilityObject*)accessibilityObject;

// Used to inform an element when a notification is posted for it. Used by DRT.
- (void)accessibilityPostedNotification:(NSString *)notification;

- (NSView*)attachmentView;

@end

#endif // AccessibilityObjectWrapper_h
