

#include "config.h"
#include "LoaderRunLoopCF.h"

#include <wtf/Threading.h>

namespace WebCore {

static CFRunLoopRef loaderRunLoopObject = 0;

static void emptyPerform(void*) 
{
}

static void* runLoaderThread(void*)
{
    loaderRunLoopObject = CFRunLoopGetCurrent();

    // Must add a source to the run loop to prevent CFRunLoopRun() from exiting.
    CFRunLoopSourceContext ctxt = {0, (void*)1 /*must be non-NULL*/, 0, 0, 0, 0, 0, 0, 0, emptyPerform};
    CFRunLoopSourceRef bogusSource = CFRunLoopSourceCreate(0, 0, &ctxt);
    CFRunLoopAddSource(loaderRunLoopObject, bogusSource, kCFRunLoopDefaultMode);

    CFRunLoopRun();

    return 0;
}

CFRunLoopRef loaderRunLoop()
{
    ASSERT(isMainThread());
    if (!loaderRunLoopObject) {
        createThread(runLoaderThread, 0, "WebCore: CFNetwork Loader");
        while (!loaderRunLoopObject) {
            // FIXME: Sleep 10? that can't be right...
            Sleep(10);
        }
    }
    return loaderRunLoopObject;
}

}
