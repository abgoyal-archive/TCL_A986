

#include "config.h"
#include "ScriptCachedFrameData.h"

#include "Frame.h"
#include "GCController.h"
#include "Page.h"
#include "PageGroup.h"
#include <runtime/JSLock.h>
#include "ScriptController.h"

using namespace JSC;

namespace WebCore {

ScriptCachedFrameData::ScriptCachedFrameData(Frame* frame)
    : m_domWindow(0)
{
    JSLock lock(SilenceAssertionsOnly);

    ScriptController* scriptController = frame->script();
    ScriptController::ShellMap& windowShells = scriptController->m_windowShells;

    ScriptController::ShellMap::iterator windowShellsEnd = windowShells.end();
    for (ScriptController::ShellMap::iterator iter = windowShells.begin(); iter != windowShellsEnd; ++iter) {
        JSDOMWindow* window = iter->second->window();
        m_windows.add(iter->first.get(), window);
        m_domWindow = window->impl();
    }

    scriptController->attachDebugger(0);
}

DOMWindow* ScriptCachedFrameData::domWindow() const
{
    return m_domWindow;
}

ScriptCachedFrameData::~ScriptCachedFrameData()
{
    clear();
}

void ScriptCachedFrameData::restore(Frame* frame)
{
    JSLock lock(SilenceAssertionsOnly);

    ScriptController* scriptController = frame->script();
    ScriptController::ShellMap& windowShells = scriptController->m_windowShells;

    ScriptController::ShellMap::iterator windowShellsEnd = windowShells.end();
    for (ScriptController::ShellMap::iterator iter = windowShells.begin(); iter != windowShellsEnd; ++iter) {
        DOMWrapperWorld* world = iter->first.get();
        JSDOMWindowShell* windowShell = iter->second.get();

        if (JSDOMWindow* window = m_windows.get(world))
            windowShell->setWindow(window);
        else {
            windowShell->setWindow(frame->domWindow());

            if (Page* page = frame->page()) {
                scriptController->attachDebugger(windowShell, page->debugger());
                windowShell->window()->setProfileGroup(page->group().identifier());
            }
        }
    }
}

void ScriptCachedFrameData::clear()
{
    if (m_windows.isEmpty())
        return;

    JSLock lock(SilenceAssertionsOnly);
    m_windows.clear();
    gcController().garbageCollectSoon();
}

} // namespace WebCore
