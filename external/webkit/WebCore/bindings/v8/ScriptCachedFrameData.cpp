

#include "config.h"
#include "ScriptCachedFrameData.h"

#include "Frame.h"
#include "ScriptController.h"
#include "V8DOMWindow.h"

namespace WebCore {

ScriptCachedFrameData::ScriptCachedFrameData(Frame* frame)
    : m_domWindow(0)
{
    v8::HandleScope handleScope;
    // The context can only be the context of the main world.
    ASSERT(V8Proxy::mainWorldContext(frame) == V8Proxy::context(frame));
    m_context.set(V8Proxy::mainWorldContext(frame));
    // The context can be 0, e.g. if JS is disabled in the browser.
    if (m_context.get().IsEmpty())
        return;
    m_global.set(m_context.get()->Global());
    m_domWindow = frame->domWindow();
}

DOMWindow* ScriptCachedFrameData::domWindow() const
{
    return m_domWindow;
}

void ScriptCachedFrameData::restore(Frame* frame)
{
    if (m_context.get().IsEmpty())
        return;

    v8::HandleScope handleScope;
    v8::Context::Scope contextScope(m_context.get());

    m_context.get()->ReattachGlobal(m_global.get());
    V8Proxy* proxy = V8Proxy::retrieve(frame);
    if (proxy)
        proxy->windowShell()->setContext(m_context.get());
}

void ScriptCachedFrameData::clear()
{
    m_context.clear();
    m_global.clear();
}

} // namespace WebCore
