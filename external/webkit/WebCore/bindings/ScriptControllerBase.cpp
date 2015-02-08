

#include "config.h"
#include "ScriptController.h"

#include "Frame.h"
#include "FrameLoaderClient.h"
#include "Page.h"
#include "ScriptSourceCode.h"
#include "ScriptValue.h"
#include "Settings.h"
#include "XSSAuditor.h"

namespace WebCore {

bool ScriptController::canExecuteScripts()
{
    // FIXME: We should get this information from the document instead of the frame.
    if (m_frame->loader()->isSandboxed(SandboxScripts))
        return false;

    Settings* settings = m_frame->settings();
    return m_frame->loader()->client()->allowJavaScript(settings && settings->isJavaScriptEnabled());
}

ScriptValue ScriptController::executeScript(const String& script, bool forceUserGesture)
{
    return executeScript(ScriptSourceCode(script, forceUserGesture ? KURL() : m_frame->loader()->url()));
}

ScriptValue ScriptController::executeScript(const ScriptSourceCode& sourceCode)
{
    if (!canExecuteScripts() || isPaused())
        return ScriptValue();

    bool wasInExecuteScript = m_inExecuteScript;
    m_inExecuteScript = true;

    ScriptValue result = evaluate(sourceCode);

    if (!wasInExecuteScript) {
        m_inExecuteScript = false;
        Document::updateStyleForAllDocuments();
    }

    return result;
}


bool ScriptController::executeIfJavaScriptURL(const KURL& url, bool userGesture, bool replaceDocument)
{
    if (!protocolIsJavaScript(url))
        return false;

    if (m_frame->page() && !m_frame->page()->javaScriptURLsAreAllowed())
        return true;

    if (m_frame->inViewSourceMode())
        return true;

    const int javascriptSchemeLength = sizeof("javascript:") - 1;

    String script = decodeURLEscapeSequences(url.string().substring(javascriptSchemeLength));
    ScriptValue result;
    if (xssAuditor()->canEvaluateJavaScriptURL(script))
        result = executeScript(script, userGesture);

    String scriptResult;
#if USE(JSC)
    JSDOMWindowShell* shell = windowShell(mainThreadNormalWorld());
    JSC::ExecState* exec = shell->window()->globalExec();
    if (!result.getString(exec, scriptResult))
        return true;
#else
    if (!result.getString(scriptResult))
        return true;
#endif

    // FIXME: We should always replace the document, but doing so
    //        synchronously can cause crashes:
    //        http://bugs.webkit.org/show_bug.cgi?id=16782
    if (replaceDocument) 
        m_frame->loader()->replaceDocument(scriptResult);

    return true;
}

} // namespace WebCore
