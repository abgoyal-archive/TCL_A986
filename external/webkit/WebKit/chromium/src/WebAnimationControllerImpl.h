

#ifndef WebAnimationControllerImpl_h
#define WebAnimationControllerImpl_h

// FIXME: This relative path is a temporary hack to support using this
// header from webkit/glue.
#include "../public/WebAnimationController.h"

namespace WebCore {
class AnimationController;
}

namespace WebKit {
class WebFrameImpl;

class WebAnimationControllerImpl : public WebAnimationController {
public:
    explicit WebAnimationControllerImpl(WebFrameImpl*);
    virtual ~WebAnimationControllerImpl() { }

    virtual bool pauseAnimationAtTime(WebElement&,
                                      const WebString& animationName,
                                      double time);
    virtual bool pauseTransitionAtTime(WebElement&,
                                       const WebString& propertyName,
                                       double time);
    virtual unsigned numberOfActiveAnimations() const;
private:
    WebFrameImpl* m_frameImpl;
    WebCore::AnimationController* animationController() const;
};

} // namespace WebKit

#endif
