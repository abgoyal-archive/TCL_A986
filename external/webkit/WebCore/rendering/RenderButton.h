

#ifndef RenderButton_h
#define RenderButton_h

#include "RenderFlexibleBox.h"
#include "Timer.h"
#include <wtf/OwnPtr.h>

namespace WebCore {

class RenderTextFragment;

// RenderButtons are just like normal flexboxes except that they will generate an anonymous block child.
// For inputs, they will also generate an anonymous RenderText and keep its style and content up
// to date as the button changes.
class RenderButton : public RenderFlexibleBox {
public:
    RenderButton(Node*);

    virtual const char* renderName() const { return "RenderButton"; }
    virtual bool isRenderButton() const { return true; }

    virtual void addChild(RenderObject* newChild, RenderObject *beforeChild = 0);
    virtual void removeChild(RenderObject*);
    virtual void removeLeftoverAnonymousBlock(RenderBlock*) { }
    virtual bool createsAnonymousWrapper() const { return true; }

    void setupInnerStyle(RenderStyle*);
    virtual void updateFromElement();

    virtual void updateBeforeAfterContent(PseudoId);

    virtual bool hasControlClip() const { return true; }
    virtual IntRect controlClipRect(int /*tx*/, int /*ty*/) const;

    void setText(const String&);
    String text() const;

    virtual bool canHaveChildren() const;

protected:
    virtual void styleWillChange(StyleDifference, const RenderStyle* newStyle);
    virtual void styleDidChange(StyleDifference, const RenderStyle* oldStyle);

    virtual bool hasLineIfEmpty() const { return true; }

    void timerFired(Timer<RenderButton>*);

    RenderTextFragment* m_buttonText;
    RenderBlock* m_inner;

    OwnPtr<Timer<RenderButton> > m_timer;
    bool m_default;
};

inline RenderButton* toRenderButton(RenderObject* object)
{ 
    ASSERT(!object || object->isRenderButton());
    return static_cast<RenderButton*>(object);
}

inline const RenderButton* toRenderButton(const RenderObject* object)
{ 
    ASSERT(!object || object->isRenderButton());
    return static_cast<const RenderButton*>(object);
}

// This will catch anyone doing an unnecessary cast.
void toRenderButton(const RenderButton*);

} // namespace WebCore

#endif // RenderButton_h
