

#include "config.h"

#if ENABLE(RUBY)
#include "RenderRubyText.h"

namespace WebCore {

RenderRubyText::RenderRubyText(Node* node)
    : RenderBlock(node)
{
}

RenderRubyText::~RenderRubyText()
{
}

bool RenderRubyText::isChildAllowed(RenderObject* child, RenderStyle*) const
{
    return child->isInline();
}

} // namespace WebCore

#endif
