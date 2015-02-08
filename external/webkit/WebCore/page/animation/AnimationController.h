

#ifndef AnimationController_h
#define AnimationController_h

#include "CSSPropertyNames.h"
#include <wtf/Forward.h>

namespace WebCore {

class AnimationBase;
class AnimationControllerPrivate;
class AtomicString;
class Document;
class Element;
class Frame;
class Node;
class RenderObject;
class RenderStyle;
class String;

class AnimationController {
public:
    AnimationController(Frame*);
    ~AnimationController();

    void cancelAnimations(RenderObject*);
    PassRefPtr<RenderStyle> updateAnimations(RenderObject*, RenderStyle* newStyle);
    PassRefPtr<RenderStyle> getAnimatedStyleForRenderer(RenderObject*);

    // This is called when an accelerated animation or transition has actually started to animate.
    void notifyAnimationStarted(RenderObject*, double startTime);

    bool pauseAnimationAtTime(RenderObject*, const String& name, double t); // To be used only for testing
    bool pauseTransitionAtTime(RenderObject*, const String& property, double t); // To be used only for testing
    unsigned numberOfActiveAnimations() const; // To be used only for testing
    
    bool isAnimatingPropertyOnRenderer(RenderObject*, CSSPropertyID, bool isRunningNow = true) const;

    void suspendAnimations(Document*);
    void resumeAnimations(Document*);

    void beginAnimationUpdate();
    void endAnimationUpdate();
    
    static bool supportsAcceleratedAnimationOfProperty(CSSPropertyID);

private:
    AnimationControllerPrivate* m_data;
};

} // namespace WebCore

#endif // AnimationController_h
