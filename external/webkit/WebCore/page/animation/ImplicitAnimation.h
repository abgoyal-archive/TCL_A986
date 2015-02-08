

#ifndef ImplicitAnimation_h
#define ImplicitAnimation_h

#include "AnimationBase.h"
#include "Document.h"

namespace WebCore {

// An ImplicitAnimation tracks the state of a transition of a specific CSS property
// for a single RenderObject.
class ImplicitAnimation : public AnimationBase {
public:
    static PassRefPtr<ImplicitAnimation> create(const Animation* animation, int animatingProperty, RenderObject* renderer, CompositeAnimation* compositeAnimation, RenderStyle* fromStyle)
    {
        return adoptRef(new ImplicitAnimation(animation, animatingProperty, renderer, compositeAnimation, fromStyle));
    };
    
    int transitionProperty() const { return m_transitionProperty; }
    int animatingProperty() const { return m_animatingProperty; }

    virtual void onAnimationEnd(double elapsedTime);
    virtual bool startAnimation(double timeOffset);
    virtual void pauseAnimation(double /*timeOffset*/) { }
    virtual void endAnimation();

    virtual void animate(CompositeAnimation*, RenderObject*, const RenderStyle* currentStyle, RenderStyle* targetStyle, RefPtr<RenderStyle>& animatedStyle);
    virtual void getAnimatedStyle(RefPtr<RenderStyle>& animatedStyle);
    virtual void reset(RenderStyle* to);

    void setOverridden(bool);
    virtual bool overridden() const { return m_overridden; }

    virtual bool affectsProperty(int) const;

    bool hasStyle() const { return m_fromStyle && m_toStyle; }

    bool isTargetPropertyEqual(int, const RenderStyle* targetStyle);

    void blendPropertyValueInStyle(int, RenderStyle* currentStyle);

    virtual double timeToNextService();
    
    bool active() const { return m_active; }
    void setActive(bool b) { m_active = b; }

protected:
    bool shouldSendEventForListener(Document::ListenerType) const;    
    bool sendTransitionEvent(const AtomicString&, double elapsedTime);

    void validateTransformFunctionList();

private:
    ImplicitAnimation(const Animation*, int animatingProperty, RenderObject*, CompositeAnimation*, RenderStyle* fromStyle);    
    virtual ~ImplicitAnimation();

    int m_transitionProperty;   // Transition property as specified in the RenderStyle. May be cAnimateAll
    int m_animatingProperty;    // Specific property for this ImplicitAnimation
    bool m_overridden;          // true when there is a keyframe animation that overrides the transitioning property
    bool m_active;              // used for culling the list of transitions

    // The two styles that we are blending.
    RefPtr<RenderStyle> m_fromStyle;
    RefPtr<RenderStyle> m_toStyle;
};

} // namespace WebCore

#endif // ImplicitAnimation_h
