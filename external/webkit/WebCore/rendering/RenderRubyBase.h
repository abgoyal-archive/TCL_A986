

#ifndef RenderRubyBase_h
#define RenderRubyBase_h

#if ENABLE(RUBY)

#include "RenderBlock.h"

namespace WebCore {

class RenderRubyBase : public RenderBlock {
public:
    RenderRubyBase(Node*);
    virtual ~RenderRubyBase();

    virtual const char* renderName() const { return "RenderRubyBase (anonymous)"; }

    virtual bool isRubyBase() const { return true; }

    virtual bool isChildAllowed(RenderObject*, RenderStyle*) const;

private:
    bool hasOnlyWrappedInlineChildren(RenderObject* beforeChild = 0) const;

    void moveChildren(RenderRubyBase* toBase, RenderObject* fromBeforeChild = 0);
    void moveInlineChildren(RenderRubyBase* toBase, RenderObject* fromBeforeChild = 0);
    void moveBlockChildren(RenderRubyBase* toBase, RenderObject* fromBeforeChild = 0);
    void mergeBlockChildren(RenderRubyBase* toBase, RenderObject* fromBeforeChild = 0);
    
    // Allow RenderRubyRun to manipulate the children within ruby bases.
    friend class RenderRubyRun;
};

} // namespace WebCore

#endif

#endif // RenderRubyBase_h
