

#ifndef RenderMathMLBlock_h
#define RenderMathMLBlock_h

#if ENABLE(MATHML)

#include "RenderBlock.h"

#define ENABLE_DEBUG_MATH_LAYOUT 0

namespace WebCore {
    
class RenderMathMLBlock : public RenderBlock {
public:
    RenderMathMLBlock(Node* container);
    virtual bool isChildAllowed(RenderObject*, RenderStyle*) const;
    
    virtual bool isRenderMathMLBlock() const { return true; }
    virtual bool isRenderMathMLOperator() const { return false; }
    virtual bool isRenderMathMLRow() const { return false; }
    virtual bool isRenderMathMLMath() const { return false; }
    virtual bool hasBase() const { return false; }
    virtual int nonOperatorHeight() const;
    virtual void stretchToHeight(int height);

#if ENABLE(DEBUG_MATH_LAYOUT)
    virtual void paint(PaintInfo&, int tx, int ty);
#endif
    
protected:
    virtual PassRefPtr<RenderStyle> makeBlockStyle();
    
};

inline RenderMathMLBlock* toRenderMathMLBlock(RenderObject* object)
{ 
    ASSERT(!object || object->isRenderMathMLBlock());
    return static_cast<RenderMathMLBlock*>(object);
}

inline const RenderMathMLBlock* toRenderMathMLBlock(const RenderObject* object)
{ 
    ASSERT(!object || object->isRenderMathMLBlock());
    return static_cast<const RenderMathMLBlock*>(object);
}
    
}


#endif // ENABLE(MATHML)
#endif // RenderMathMLBlock_h
