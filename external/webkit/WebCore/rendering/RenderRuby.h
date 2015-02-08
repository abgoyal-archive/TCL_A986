

#ifndef RenderRuby_h
#define RenderRuby_h

#if ENABLE(RUBY)

#include "RenderBlock.h"
#include "RenderInline.h"

namespace WebCore {

// Following the HTML 5 spec, the box object model for a <ruby> element allows several runs of ruby
// bases with their respective ruby texts looks as follows:
//
// 1 RenderRuby object, corresponding to the whole <ruby> HTML element
//      1+ RenderRubyRun (anonymous)
//          0 or 1 RenderRubyText - shuffled to the front in order to re-use existing block layouting
//              0-n inline object(s)
//          0 or 1 RenderRubyBase - contains the inline objects that make up the ruby base
//              1-n inline object(s)
//
// Note: <rp> elements are defined as having 'display:none' and thus normally are not assigned a renderer.

// <ruby> when used as 'display:inline'
class RenderRubyAsInline : public RenderInline {
public:
    RenderRubyAsInline(Node*);
    virtual ~RenderRubyAsInline();

    virtual bool isChildAllowed(RenderObject*, RenderStyle*) const;
    virtual void addChild(RenderObject* child, RenderObject* beforeChild = 0);
    virtual void removeChild(RenderObject* child);

private:
    virtual bool isRuby() const { return true; }
    virtual const char* renderName() const { return "RenderRuby (inline)"; }
    virtual bool createsAnonymousWrapper() const { return true; }
    virtual void removeLeftoverAnonymousBlock(RenderBlock*) { ASSERT_NOT_REACHED(); }
};

// <ruby> when used as 'display:block' or 'display:inline-block'
class RenderRubyAsBlock : public RenderBlock {
public:
    RenderRubyAsBlock(Node*);
    virtual ~RenderRubyAsBlock();

    virtual bool isChildAllowed(RenderObject*, RenderStyle*) const;
    virtual void addChild(RenderObject* child, RenderObject* beforeChild = 0);
    virtual void removeChild(RenderObject* child);

private:
    virtual bool isRuby() const { return true; }
    virtual const char* renderName() const { return "RenderRuby (block)"; }
    virtual bool createsAnonymousWrapper() const { return true; }
    virtual void removeLeftoverAnonymousBlock(RenderBlock*) { ASSERT_NOT_REACHED(); }
};

} // namespace WebCore

#endif

#endif // RenderRuby_h
