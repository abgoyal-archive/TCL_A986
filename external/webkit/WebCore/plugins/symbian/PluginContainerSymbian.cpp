

#include "config.h"
#include "PluginContainerSymbian.h"

#include "FocusController.h"
#include "Frame.h"
#include "FrameView.h"
#include "Page.h"
#include "PlatformKeyboardEvent.h"
#include "PluginView.h"

#include <QApplication>
#include <QWidget>

using namespace WebCore;

PluginContainerSymbian::PluginContainerSymbian(PluginView* view, QWidget* parent)
    : m_parent(parent)
    , m_pluginView(view)
    , m_hasPendingGeometryChange(false)
{
    setParent(m_parent);
}

PluginContainerSymbian::~PluginContainerSymbian()
{
}

void PluginContainerSymbian::requestGeometry(const QRect& rect, const QRegion& clip)
{
    if (m_windowRect != rect || m_clipRegion != clip) {
        m_windowRect = rect;
        m_clipRegion = clip;
        m_hasPendingGeometryChange = true;
    }
}

void PluginContainerSymbian::adjustGeometry()
{
    if (m_hasPendingGeometryChange) {
        setGeometry(m_windowRect);
        setMask(m_clipRegion);
        m_hasPendingGeometryChange = false;
    }
}

void PluginContainerSymbian::focusInEvent(QFocusEvent* event)
{
    if (Page* page = m_pluginView->parentFrame()->page())
        page->focusController()->setActive(true);

    m_pluginView->focusPluginElement();
}

void PluginContainerSymbian::focusOutEvent(QFocusEvent*)
{
    if (Page* page = m_pluginView->parentFrame()->page())
        page->focusController()->setActive(false);
}
