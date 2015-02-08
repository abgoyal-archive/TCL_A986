

#ifndef CompositeAnimation_h
#define CompositeAnimation_h

#include "AtomicString.h"

#include "ImplicitAnimation.h"
#include "KeyframeAnimation.h"
#include <wtf/HashMap.h>
#include <wtf/Noncopyable.h>

namespace WebCore {

class AnimationControllerPrivate;
class AnimationController;
class RenderObject;
class RenderStyle;

// A CompositeAnimation represents a collection of animations that are running
// on a single RenderObject, such as a number of properties transitioning at once.
class CompositeAnimation : public RefCounted<CompositeAnimation> {
public:
    static PassRefPtr<CompositeAnimation> create(AnimationControllerPrivate* animationController)
    {
        return adoptRef(new CompositeAnimation(animationController));
    };

    ~CompositeAnimation();
    
    void clearRenderer();

    PassRefPtr<RenderStyle> animate(RenderObject*, RenderStyle* currentStyle, RenderStyle* targetStyle);
    PassRefPtr<RenderStyle> getAnimatedStyle() const;

    double timeToNextService() const;
    
    AnimationControllerPrivate* animationController() const { return m_animationController; }

    void suspendAnimations();
    void resumeAnimations();
    bool isSuspended() const { return m_isSuspended; }
    
    bool hasAnimations() const  { return !m_transitions.isEmpty() || !m_keyframeAnimations.isEmpty(); }

    void setAnimating(bool);
    bool isAnimatingProperty(int property, bool isRunningNow) const;
    
    PassRefPtr<KeyframeAnimation> getAnimationForProperty(int property) const;

    void overrideImplicitAnimations(int property);
    void resumeOverriddenImplicitAnimations(int property);

    bool pauseAnimationAtTime(const AtomicString& name, double t);
    bool pauseTransitionAtTime(int property, double t);
    unsigned numberOfActiveAnimations() const;

private:
    CompositeAnimation(AnimationControllerPrivate* animationController)
        : m_animationController(animationController)
        , m_numStyleAvailableWaiters(0)
        , m_isSuspended(false)
    {
    }

    void updateTransitions(RenderObject*, RenderStyle* currentStyle, RenderStyle* targetStyle);
    void updateKeyframeAnimations(RenderObject*, RenderStyle* currentStyle, RenderStyle* targetStyle);
    
    typedef HashMap<int, RefPtr<ImplicitAnimation> > CSSPropertyTransitionsMap;
    typedef HashMap<AtomicStringImpl*, RefPtr<KeyframeAnimation> >  AnimationNameMap;

    AnimationControllerPrivate* m_animationController;
    CSSPropertyTransitionsMap m_transitions;
    AnimationNameMap m_keyframeAnimations;
    Vector<AtomicStringImpl*> m_keyframeAnimationOrderMap;
    unsigned m_numStyleAvailableWaiters;
    bool m_isSuspended;
};

} // namespace WebCore

#endif // CompositeAnimation_h
