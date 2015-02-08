

#include "config.h"
#include "InspectorFrontendHost.h"

#if ENABLE(INSPECTOR)

#include "ContextMenu.h"
#include "ContextMenuItem.h"
#include "ContextMenuController.h"
#include "ContextMenuProvider.h"
#include "Element.h"
#include "Frame.h"
#include "FrameLoader.h"
#include "HitTestResult.h"
#include "HTMLFrameOwnerElement.h"
#include "InspectorClient.h"
#include "InspectorFrontend.h"
#include "InspectorResource.h"
#include "Page.h"
#include "Pasteboard.h"

#include <wtf/RefPtr.h>
#include <wtf/StdLibExtras.h>

using namespace std;

namespace WebCore {

InspectorFrontendHost::InspectorFrontendHost(InspectorController* inspectorController, InspectorClient* client)
    : m_inspectorController(inspectorController)
    , m_client(client)
{
}

InspectorFrontendHost::~InspectorFrontendHost()
{
    if (m_menuProvider)
        m_menuProvider->disconnect();
}

void InspectorFrontendHost::loaded()
{
    if (m_inspectorController)
        m_inspectorController->scriptObjectReady();
}

void InspectorFrontendHost::attach()
{
    if (m_inspectorController)
        m_inspectorController->attachWindow();
}

void InspectorFrontendHost::detach()
{
    if (m_inspectorController)
        m_inspectorController->detachWindow();
}

void InspectorFrontendHost::closeWindow()
{
    if (m_inspectorController)
        m_inspectorController->closeWindow();
}

void InspectorFrontendHost::windowUnloading()
{
    if (m_inspectorController)
        m_inspectorController->close();
}

void InspectorFrontendHost::setAttachedWindowHeight(unsigned height)
{
    if (m_inspectorController)
        m_inspectorController->setAttachedWindowHeight(height);
}

void InspectorFrontendHost::moveWindowBy(float x, float y) const
{
    if (m_inspectorController)
        m_inspectorController->moveWindowBy(x, y);
}

String InspectorFrontendHost::localizedStringsURL()
{
    return m_client->localizedStringsURL();
}

String InspectorFrontendHost::hiddenPanels()
{
    return m_client->hiddenPanels();
}

const String& InspectorFrontendHost::platform() const
{
#if PLATFORM(MAC)
    DEFINE_STATIC_LOCAL(const String, platform, ("mac"));
#elif OS(WINDOWS)
    DEFINE_STATIC_LOCAL(const String, platform, ("windows"));
#elif OS(LINUX)
    DEFINE_STATIC_LOCAL(const String, platform, ("linux"));
#else
    DEFINE_STATIC_LOCAL(const String, platform, ("unknown"));
#endif
    return platform;
}

const String& InspectorFrontendHost::port() const
{
#if PLATFORM(QT)
    DEFINE_STATIC_LOCAL(const String, port, ("qt"));
#elif PLATFORM(GTK)
    DEFINE_STATIC_LOCAL(const String, port, ("gtk"));
#elif PLATFORM(WX)
    DEFINE_STATIC_LOCAL(const String, port, ("wx"));
#else
    DEFINE_STATIC_LOCAL(const String, port, ("unknown"));
#endif

    return port;
}

void InspectorFrontendHost::copyText(const String& text)
{
    Pasteboard::generalPasteboard()->writePlainText(text);
}

void InspectorFrontendHost::showContextMenu(Event* event, const Vector<ContextMenuItem*>& items)
{
    if (!m_inspectorController)
        return;
    if (!m_inspectorController->windowVisible())
        return;


    m_menuProvider = MenuProvider::create(this, items);
    ContextMenuController* menuController = m_inspectorController->m_page->contextMenuController();
    menuController->showContextMenu(event, m_menuProvider);
}

void InspectorFrontendHost::contextMenuItemSelected(ContextMenuItem* item)
{
    if (m_inspectorController && m_inspectorController->windowVisible()) {
        int itemNumber = item->action() - ContextMenuItemBaseCustomTag;
        m_inspectorController->m_frontend->contextMenuItemSelected(itemNumber);
    }
}

void InspectorFrontendHost::contextMenuCleared()
{
    m_menuProvider = 0;
    if (m_inspectorController && m_inspectorController->windowVisible())
        m_inspectorController->m_frontend->contextMenuCleared();
}

} // namespace WebCore

#endif // ENABLE(INSPECTOR)
