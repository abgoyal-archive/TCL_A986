

#include "config.h"
#include "AccessibilityScrollbar.h"

#include "Scrollbar.h"

namespace WebCore {

AccessibilityScrollbar::AccessibilityScrollbar()
    : m_scrollbar(0)
{
}

PassRefPtr<AccessibilityScrollbar> AccessibilityScrollbar::create()
{
    return adoptRef(new AccessibilityScrollbar);
}

float AccessibilityScrollbar::valueForRange() const
{
    if (!m_scrollbar)
        return 0;
    return m_scrollbar->currentPos();
}

} // namespace WebCore
