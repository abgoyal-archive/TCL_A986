

#ifndef AnimationControllerPrivate_h
#define AnimationControllerPrivate_h

#include "AtomicString.h"
#include "CSSPropertyNames.h"
#include "PlatformString.h"
#include "Timer.h"
#include <wtf/HashMap.h>
#include <wtf/PassRefPtr.h>
#include <wtf/RefPtr.h>
#include <wtf/Vector.h>

namespace WebCore {

class AnimationBase;
class CompositeAnimation;
class Document;
class Element;
class Frame;
class Node;
class RenderObject;
class RenderStyle;

class AnimationControllerPrivate : public Noncopyable {
public:
    AnimationControllerPrivate(Frame*);
    ~AnimationControllerPrivate();

    PassRefPtr<CompositeAnimation> accessCompositeAnimation(RenderObject*);
    bool clear(RenderObject*);

    void animationTimerFired(Timer<AnimationControllerPrivate>*);
    void updateAnimationTimer(bool callSetChanged = false);

    void updateStyleIfNeededDispatcherFired(Timer<AnimationControllerPrivate>*);
    void startUpdateStyleIfNeededDispatcher();
    void addEventToDispatch(PassRefPtr<Element> element, const AtomicString& eventType, const String& name, double elapsedTime);
    void addNodeChangeToDispatch(PassRefPtr<Node>);

    bool hasAnimations() const { return !m_compositeAnimations.isEmpty(); }

    void suspendAnimations(Document*);
    void resumeAnimations(Document*);

    bool isAnimatingPropertyOnRenderer(RenderObject*, CSSPropertyID, bool isRunningNow) const;

    bool pauseAnimationAtTime(RenderObject*, const String& name, double t);
    bool pauseTransitionAtTime(RenderObject*, const String& property, double t);
    unsigned numberOfActiveAnimations() const;

    PassRefPtr<RenderStyle> getAnimatedStyleForRenderer(RenderObject* renderer);

    double beginAnimationUpdateTime();
    void setBeginAnimationUpdateTime(double t) { m_beginAnimationUpdateTime = t; }
    void endAnimationUpdate();
    void receivedStartTimeResponse(double);
    
    void addToStyleAvailableWaitList(AnimationBase*);
    void removeFromStyleAvailableWaitList(AnimationBase*);    
    
    void addToStartTimeResponseWaitList(AnimationBase*, bool willGetResponse);
    void removeFromStartTimeResponseWaitList(AnimationBase*);    
    void startTimeResponse(double t);
    
private:
    void styleAvailable();

    typedef HashMap<RenderObject*, RefPtr<CompositeAnimation> > RenderObjectAnimationMap;

    RenderObjectAnimationMap m_compositeAnimations;
    Timer<AnimationControllerPrivate> m_animationTimer;
    Timer<AnimationControllerPrivate> m_updateStyleIfNeededDispatcher;
    Frame* m_frame;
    
    class EventToDispatch {
    public:
        RefPtr<Element> element;
        AtomicString eventType;
        String name;
        double elapsedTime;
    };
    
    Vector<EventToDispatch> m_eventsToDispatch;
    Vector<RefPtr<Node> > m_nodeChangesToDispatch;
    
    double m_beginAnimationUpdateTime;
    AnimationBase* m_styleAvailableWaiters;
    AnimationBase* m_lastStyleAvailableWaiter;
    
    AnimationBase* m_responseWaiters;
    AnimationBase* m_lastResponseWaiter;
    bool m_waitingForResponse;
};

} // namespace WebCore

#endif // AnimationControllerPrivate_h
