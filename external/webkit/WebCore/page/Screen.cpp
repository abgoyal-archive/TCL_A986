


#include "config.h"
#include "Screen.h"

#include "FloatRect.h"
#include "Frame.h"
#include "FrameView.h"
#include "PlatformScreen.h"
#include "Widget.h"

namespace WebCore {

Screen::Screen(Frame* frame)
    : m_frame(frame)
{
}

Frame* Screen::frame() const
{
    return m_frame;
}

void Screen::disconnectFrame()
{
    m_frame = 0;
}

unsigned Screen::height() const
{
    if (!m_frame)
        return 0;
    return static_cast<unsigned>(screenRect(m_frame->view()).height());
}

unsigned Screen::width() const
{
    if (!m_frame)
        return 0;
    return static_cast<unsigned>(screenRect(m_frame->view()).width());
}

unsigned Screen::colorDepth() const
{
    if (!m_frame)
        return 0;
    return static_cast<unsigned>(screenDepth(m_frame->view()));
}

unsigned Screen::pixelDepth() const
{
    if (!m_frame)
        return 0;
    return static_cast<unsigned>(screenDepth(m_frame->view()));
}

unsigned Screen::availLeft() const
{
    if (!m_frame)
        return 0;
    return static_cast<unsigned>(screenAvailableRect(m_frame->view()).x());
}

unsigned Screen::availTop() const
{
    if (!m_frame)
        return 0;
    return static_cast<unsigned>(screenAvailableRect(m_frame->view()).y());
}

unsigned Screen::availHeight() const
{
    if (!m_frame)
        return 0;
    return static_cast<unsigned>(screenAvailableRect(m_frame->view()).height());
}

unsigned Screen::availWidth() const
{
    if (!m_frame)
        return 0;
    return static_cast<unsigned>(screenAvailableRect(m_frame->view()).width());
}

} // namespace WebCore
