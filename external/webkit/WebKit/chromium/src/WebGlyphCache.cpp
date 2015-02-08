

#include "config.h"
#include "WebGlyphCache.h"

#include "GlyphPageTreeNode.h"

using namespace WebCore;

namespace WebKit {

size_t WebGlyphCache::pageCount()
{
    return GlyphPageTreeNode::treeGlyphPageCount();
}

} // namespace WebKit
