

#ifndef RenderRubyRun_h
#define RenderRubyRun_h

#if ENABLE(RUBY)

#include "RenderBlock.h"

namespace WebCore {

class RenderRubyBase;
class RenderRubyText;

// RenderRubyRun are 'inline-block/table' like objects,and wrap a single pairing of a ruby base with its ruby text(s).
// See RenderRuby.h for further comments on the structure

class RenderRubyRun : public RenderBlock {
public:
    RenderRubyRun(Node*);
    virtual ~RenderRubyRun();

    virtual void destroy();

    bool hasRubyText() const;
    bool hasRubyBase() const;
    bool isEmpty() const;
    RenderRubyText* rubyText() const;
    RenderRubyBase* rubyBase() const;
    RenderRubyBase* rubyBaseSafe(); // creates the base if it doesn't already exist

    virtual bool isChildAllowed(RenderObject*, RenderStyle*) const;
    virtual void addChild(RenderObject* child, RenderObject* beforeChild = 0);
    virtual void removeChild(RenderObject* child);

    virtual RenderBlock* firstLineBlock() const;
    virtual void updateFirstLetter();

    static RenderRubyRun* staticCreateRubyRun(const RenderObject* parentRuby);

protected:
    RenderRubyBase* createRubyBase() const;

private:
    virtual bool isRubyRun() const { return true; }
    virtual const char* renderName() const { return "RenderRubyRun (anonymous)"; }
    virtual bool createsAnonymousWrapper() const { return true; }
    virtual void removeLeftoverAnonymousBlock(RenderBlock*) { }

    bool m_beingDestroyed;
};

} // namespace WebCore

#endif

#endif // RenderRubyRun_h
