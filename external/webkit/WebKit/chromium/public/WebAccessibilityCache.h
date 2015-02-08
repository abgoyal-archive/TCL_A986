

#ifndef WebAccessibilityCache_h
#define WebAccessibilityCache_h

#include "WebCommon.h"

namespace WebKit {

class WebView;
class WebAccessibilityObject;

class WebAccessibilityCache {
public:
    WebAccessibilityCache() {}
    virtual ~WebAccessibilityCache() {}

    static WebAccessibilityCache* create();
    static void enableAccessibility();

    virtual void initialize(WebView* view) = 0;
    virtual bool isInitialized() const = 0;

    virtual WebAccessibilityObject getObjectById(int) = 0;
    virtual bool isValidId(int) const = 0;
    virtual int addOrGetId(const WebAccessibilityObject& object) = 0;

    virtual void remove(int) = 0;
    virtual void clear() = 0;
};

} // namespace WebKit

#endif
