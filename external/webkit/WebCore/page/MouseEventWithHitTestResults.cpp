

#include "config.h"
#include "MouseEventWithHitTestResults.h"

#include "Element.h"
#include "Node.h"

// Would TargetedMouseEvent be a better name?

namespace WebCore {

MouseEventWithHitTestResults::MouseEventWithHitTestResults(const PlatformMouseEvent& event, const HitTestResult& hitTestResult)
    : m_event(event)
    , m_hitTestResult(hitTestResult)
{
}
        
Node* MouseEventWithHitTestResults::targetNode() const
{
    Node* node = m_hitTestResult.innerNode();
    if (!node)
        return 0;
    if (node->inDocument())
        return node;

    Element* element = node->parentElement();
    if (element && element->inDocument())
        return element;

    return node;
}

const IntPoint MouseEventWithHitTestResults::localPoint() const
{
    return m_hitTestResult.localPoint();
}

Scrollbar* MouseEventWithHitTestResults::scrollbar() const
{
    return m_hitTestResult.scrollbar();
}

bool MouseEventWithHitTestResults::isOverLink() const
{
    return m_hitTestResult.URLElement() && m_hitTestResult.URLElement()->isLink();
}

}
