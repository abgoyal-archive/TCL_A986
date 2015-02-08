

#include "config.h"
#include "ScriptState.h"

#include "Frame.h"
#include "JSDOMWindowBase.h"
#include "Node.h"
#include "Page.h"

namespace WebCore {

ScriptState* mainWorldScriptState(Frame* frame)
{
    JSDOMWindowShell* shell = frame->script()->windowShell(mainThreadNormalWorld());
    return shell->window()->globalExec();
}

ScriptState* scriptStateFromNode(DOMWrapperWorld* world, Node* node)
{
    if (!node)
        return 0;
    Document* document = node->document();
    if (!document)
        return 0;
    Frame* frame = document->frame();
    if (!frame)
        return 0;
    if (!frame->script()->canExecuteScripts())
        return 0;
    return frame->script()->globalObject(world)->globalExec();
}

ScriptState* scriptStateFromPage(DOMWrapperWorld* world, Page* page)
{
    return page->mainFrame()->script()->globalObject(world)->globalExec();
}

}
