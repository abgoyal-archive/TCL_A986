

#ifndef RenderRubyText_h
#define RenderRubyText_h

#if ENABLE(RUBY)

#include "RenderBlock.h"

namespace WebCore {

class RenderRubyText : public RenderBlock {
public:
    RenderRubyText(Node*);
    virtual ~RenderRubyText();

    virtual const char* renderName() const { return "RenderRubyText"; }

    virtual bool isRubyText() const { return true; }

    virtual bool isChildAllowed(RenderObject*, RenderStyle*) const;
};

} // namespace WebCore

#endif

#endif // RenderRubyText_h
