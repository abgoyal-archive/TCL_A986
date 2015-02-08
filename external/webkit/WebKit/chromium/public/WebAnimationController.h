

#ifndef WebAnimationController_h
#define WebAnimationController_h

#include "WebCommon.h"

namespace WebKit {

class WebElement;
class WebString;
class WebURL;

// WebAnimationController can be used to control animations in a frame. It is
// owned by a WebFrame and its life span is bound to that WebFrame.
class WebAnimationController {
public:
    WEBKIT_API virtual bool pauseAnimationAtTime(WebElement&,
                                                 const WebString& animationName,
                                                 double time) = 0;
    WEBKIT_API virtual bool pauseTransitionAtTime(WebElement&,
                                                  const WebString& propertyName,
                                                  double time) = 0;

    WEBKIT_API virtual unsigned numberOfActiveAnimations() const = 0;
protected:
    ~WebAnimationController() { }
};

} // namespace WebKit

#endif
