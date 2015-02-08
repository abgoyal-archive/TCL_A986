

#ifndef RenderCounter_h
#define RenderCounter_h

#include "CounterContent.h"
#include "RenderText.h"

namespace WebCore {

class CounterNode;

class RenderCounter : public RenderText {
public:
    RenderCounter(Document*, const CounterContent&);

    // Removes the reference to the CounterNode associated with this renderer
    // if its identifier matches the argument.
    // This is used to cause a counter display update when the CounterNode
    // tree for identifier changes.
    void invalidate(const AtomicString& identifier);

    static void destroyCounterNodes(RenderObject*);
    static void destroyCounterNode(RenderObject*, const AtomicString& identifier);
    static void rendererSubtreeAttached(RenderObject*);
    static void rendererStyleChanged(RenderObject*, const RenderStyle* oldStyle, const RenderStyle* newStyle);

private:
    virtual const char* renderName() const;
    virtual bool isCounter() const;
    virtual PassRefPtr<StringImpl> originalText() const;
    
    virtual void calcPrefWidths(int leadWidth);

    CounterContent m_counter;
    mutable CounterNode* m_counterNode;
};

inline RenderCounter* toRenderCounter(RenderObject* object)
{
    ASSERT(!object || object->isCounter());
    return static_cast<RenderCounter*>(object);
}

// This will catch anyone doing an unnecessary cast.
void toRenderCounter(const RenderCounter*);

} // namespace WebCore

#endif // RenderCounter_h
