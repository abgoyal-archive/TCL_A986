

#include "config.h"
#include "ScrollBehavior.h"

namespace WebCore {

const ScrollAlignment ScrollAlignment::alignCenterIfNeeded = { noScroll, alignCenter, alignToClosestEdge };
const ScrollAlignment ScrollAlignment::alignToEdgeIfNeeded = { noScroll, alignToClosestEdge, alignToClosestEdge };
const ScrollAlignment ScrollAlignment::alignCenterAlways = { alignCenter, alignCenter, alignCenter };
const ScrollAlignment ScrollAlignment::alignTopAlways = { alignTop, alignTop, alignTop };
const ScrollAlignment ScrollAlignment::alignBottomAlways = { alignBottom, alignBottom, alignBottom };

}; // namespace WebCore
