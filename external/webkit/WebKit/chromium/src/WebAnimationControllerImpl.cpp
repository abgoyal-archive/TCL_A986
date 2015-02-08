

#include "config.h"
#include "WebAnimationControllerImpl.h"

#include "AnimationController.h"
#include "Element.h"

#include "WebElement.h"
#include "WebFrameImpl.h"
#include "WebString.h"

using namespace WebCore;

namespace WebKit {

WebAnimationControllerImpl::WebAnimationControllerImpl(WebFrameImpl* frameImpl)
    : m_frameImpl(frameImpl)
{
    ASSERT(m_frameImpl);
}

AnimationController* WebAnimationControllerImpl::animationController() const
{
    if (!m_frameImpl->frame())
        return 0;
    return m_frameImpl->frame()->animation();
}

bool WebAnimationControllerImpl::pauseAnimationAtTime(WebElement& element,
                                                      const WebString& animationName,
                                                      double time)
{
    AnimationController* controller = animationController();
    if (!controller)
        return 0;
    return controller->pauseAnimationAtTime(PassRefPtr<Element>(element)->renderer(),
                                            animationName,
                                            time);
}

bool WebAnimationControllerImpl::pauseTransitionAtTime(WebElement& element,
                                                       const WebString& propertyName,
                                                       double time)
{
    AnimationController* controller = animationController();
    if (!controller)
        return 0;
    return controller->pauseTransitionAtTime(PassRefPtr<Element>(element)->renderer(),
                                             propertyName,
                                             time);
}

unsigned WebAnimationControllerImpl::numberOfActiveAnimations() const
{
    AnimationController* controller = animationController();
    if (!controller)
        return 0;
    return controller->numberOfActiveAnimations();
}

} // namespace WebKit
