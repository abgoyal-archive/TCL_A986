

#ifndef MouseEventWithHitTestResults_h
#define MouseEventWithHitTestResults_h

#include "HitTestResult.h"
#include "PlatformMouseEvent.h"

namespace WebCore {

class Scrollbar;

class MouseEventWithHitTestResults {
public:
    MouseEventWithHitTestResults(const PlatformMouseEvent&, const HitTestResult&);

    const PlatformMouseEvent& event() const { return m_event; }
    const HitTestResult& hitTestResult() const { return m_hitTestResult; }
    Node* targetNode() const;
    const IntPoint localPoint() const;
    Scrollbar* scrollbar() const;
    bool isOverLink() const;
    bool isOverWidget() const { return m_hitTestResult.isOverWidget(); }

private:
    PlatformMouseEvent m_event;
    HitTestResult m_hitTestResult;
};

} // namespace WebCore

#endif // MouseEventWithHitTestResults_h
