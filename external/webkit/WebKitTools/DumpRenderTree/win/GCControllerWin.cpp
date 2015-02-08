

#include "config.h"
#include "GCController.h"

#include "DumpRenderTree.h"
#include <WebCore/COMPtr.h>
#include <WebKit/WebKit.h>
#include <WebKit/WebKitCOMAPI.h>

void GCController::collect() const
{
    COMPtr<IWebJavaScriptCollector> collector;
    if (FAILED(WebKitCreateInstance(CLSID_WebJavaScriptCollector, 0, IID_IWebJavaScriptCollector, (void**)&collector)))
        return;
    collector->collect();
}

void GCController::collectOnAlternateThread(bool waitUntilDone) const
{
    COMPtr<IWebJavaScriptCollector> collector;
    if (FAILED(WebKitCreateInstance(CLSID_WebJavaScriptCollector, 0, IID_IWebJavaScriptCollector, (void**)&collector)))
        return;
    collector->collectOnAlternateThread(waitUntilDone ? TRUE : FALSE);
}

size_t GCController::getJSObjectCount() const
{
    COMPtr<IWebJavaScriptCollector> collector;
    if (FAILED(WebKitCreateInstance(CLSID_WebJavaScriptCollector, 0, IID_IWebJavaScriptCollector, (void**)&collector)))
        return 0;
    UINT objects = 0;
    collector->objectCount(&objects);
    return objects;
}
