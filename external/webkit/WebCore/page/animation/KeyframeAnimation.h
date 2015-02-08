

#ifndef KeyframeAnimation_h
#define KeyframeAnimation_h

#include "AnimationBase.h"
#include "Document.h"
#include "KeyframeList.h"

namespace WebCore {

class RenderStyle;

// A KeyframeAnimation tracks the state of an explicit animation
// for a single RenderObject.
class KeyframeAnimation : public AnimationBase {
public:
    static PassRefPtr<KeyframeAnimation> create(const Animation* animation, RenderObject* renderer, int index, CompositeAnimation* compositeAnimation, RenderStyle* unanimatedStyle)
    {
        return adoptRef(new KeyframeAnimation(animation, renderer, index, compositeAnimation, unanimatedStyle));
    };

    virtual void animate(CompositeAnimation*, RenderObject*, const RenderStyle* currentStyle, RenderStyle* targetStyle, RefPtr<RenderStyle>& animatedStyle);
    virtual void getAnimatedStyle(RefPtr<RenderStyle>& animatedStyle);

    const AtomicString& name() const { return m_keyframes.animationName(); }
    int index() const { return m_index; }
    void setIndex(int i) { m_index = i; }

    bool hasAnimationForProperty(int property) const;
    
    void setUnanimatedStyle(PassRefPtr<RenderStyle> style) { m_unanimatedStyle = style; }
    RenderStyle* unanimatedStyle() const { return m_unanimatedStyle.get(); }

    virtual double timeToNextService();

protected:
    virtual void onAnimationStart(double elapsedTime);
    virtual void onAnimationIteration(double elapsedTime);
    virtual void onAnimationEnd(double elapsedTime);
    virtual bool startAnimation(double timeOffset);
    virtual void pauseAnimation(double timeOffset);
    virtual void endAnimation();

    virtual void overrideAnimations();
    virtual void resumeOverriddenAnimations();

    bool shouldSendEventForListener(Document::ListenerType inListenerType) const;
    bool sendAnimationEvent(const AtomicString&, double elapsedTime);

    virtual bool affectsProperty(int) const;

    void validateTransformFunctionList();

private:
    KeyframeAnimation(const Animation* animation, RenderObject*, int index, CompositeAnimation*, RenderStyle* unanimatedStyle);
    virtual ~KeyframeAnimation();
    
    // Get the styles surrounding the current animation time and the progress between them
    void getKeyframeAnimationInterval(const RenderStyle*& fromStyle, const RenderStyle*& toStyle, double& progress) const;

    // The keyframes that we are blending.
    KeyframeList m_keyframes;

    // The order in which this animation appears in the animation-name style.
    int m_index;

    // The style just before we started animation
    RefPtr<RenderStyle> m_unanimatedStyle;
};

} // namespace WebCore

#endif // KeyframeAnimation_h
