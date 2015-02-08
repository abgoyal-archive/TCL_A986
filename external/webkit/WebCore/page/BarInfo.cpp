

#include "config.h"
#include "BarInfo.h"

#include "Chrome.h"
#include "Frame.h"
#include "Page.h"

namespace WebCore {

BarInfo::BarInfo(Frame* frame, Type type)
    : m_frame(frame)
    , m_type(type)
{
}

Frame* BarInfo::frame() const
{
    return m_frame;
}

void BarInfo::disconnectFrame()
{
    m_frame = 0;
}

BarInfo::Type BarInfo::type() const
{
    return m_type;
}

bool BarInfo::visible() const
{
    if (!m_frame)
        return false;
    Page* page = m_frame->page();
    if (!page)
        return false;

    switch (m_type) {
    case Locationbar:
    case Personalbar:
    case Toolbar:
        return page->chrome()->toolbarsVisible();
    case Menubar:
        return page->chrome()->menubarVisible();
    case Scrollbars:
        return page->chrome()->scrollbarsVisible();
    case Statusbar:
        return page->chrome()->statusbarVisible();
    }

    ASSERT_NOT_REACHED();
    return false;
}

} // namespace WebCore
