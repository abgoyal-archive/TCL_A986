

#include "config.h"

#include "PluginInfoStore.h"
#include "PluginMainThreadScheduler.h"
#include "PluginView.h"
#include "npruntime_internal.h"

using namespace WebCore;

// The plugin view is always the ndata of the instance,. Sometimes, plug-ins will call an instance-specific function
// with a NULL instance. To workaround this, call the last plug-in view that made a call to a plug-in.
// Currently, the current plug-in view is only set before NPP_New in PluginView::start.
// This specifically works around Flash and Shockwave. When we call NPP_New, they call NPN_Useragent with a NULL instance.
static PluginView* pluginViewForInstance(NPP instance)
{
    if (instance && instance->ndata)
        return static_cast<PluginView*>(instance->ndata);
    return PluginView::currentPluginView();
}

void* NPN_MemAlloc(uint32 size)
{
    return malloc(size);
}

void NPN_MemFree(void* ptr)
{
    free(ptr);
}

uint32 NPN_MemFlush(uint32 size)
{
    // Do nothing
    return 0;
}

void NPN_ReloadPlugins(NPBool reloadPages)
{
    refreshPlugins(reloadPages);
}

NPError NPN_RequestRead(NPStream* stream, NPByteRange* rangeList)
{
    return NPERR_STREAM_NOT_SEEKABLE;
}

NPError NPN_GetURLNotify(NPP instance, const char* url, const char* target, void* notifyData)
{
    PluginView* view = pluginViewForInstance(instance);
    if(!view)
       return NPERR_INVALID_INSTANCE_ERROR;
    return view->getURLNotify(url, target, notifyData);
}

NPError NPN_GetURL(NPP instance, const char* url, const char* target)
{
    PluginView* view = pluginViewForInstance(instance);
    if(!view)
       return NPERR_INVALID_INSTANCE_ERROR;
    return view->getURL(url, target);
}

NPError NPN_PostURLNotify(NPP instance, const char* url, const char* target, uint32 len, const char* buf, NPBool file, void* notifyData)
{
    PluginView* view = pluginViewForInstance(instance);
    if(!view)
       return NPERR_INVALID_INSTANCE_ERROR;
    return view->postURLNotify(url, target, len, buf, file, notifyData);
}

NPError NPN_PostURL(NPP instance, const char* url, const char* target, uint32 len, const char* buf, NPBool file)
{
    PluginView* view = pluginViewForInstance(instance);
    if(!view)
       return NPERR_INVALID_INSTANCE_ERROR;
    return view->postURL(url, target, len, buf, file);
}

NPError NPN_NewStream(NPP instance, NPMIMEType type, const char* target, NPStream** stream)
{
    PluginView* view = pluginViewForInstance(instance);
    if(!view)
       return NPERR_INVALID_INSTANCE_ERROR;
    return view->newStream(type, target, stream);
}

int32 NPN_Write(NPP instance, NPStream* stream, int32 len, void* buffer)
{
    PluginView* view = pluginViewForInstance(instance);
    if(!view)
       return 0;
    return view->write(stream, len, buffer);
}

NPError NPN_DestroyStream(NPP instance, NPStream* stream, NPReason reason)
{
    PluginView* view = pluginViewForInstance(instance);
    if(!view)
       return NPERR_INVALID_INSTANCE_ERROR;
    return view->destroyStream(stream, reason);
}

const char* NPN_UserAgent(NPP instance)
{
    PluginView* view = pluginViewForInstance(instance);

     if (!view)
         return PluginView::userAgentStatic();
 
    return view->userAgent();
}

void NPN_Status(NPP instance, const char* message)
{
    PluginView* view = pluginViewForInstance(instance);
    if(!view)
       return;
    view->status(message);
}

void NPN_InvalidateRect(NPP instance, NPRect* invalidRect)
{
    PluginView* view = pluginViewForInstance(instance);
    if(!view)
       return;
    view->invalidateRect(invalidRect);
}

void NPN_InvalidateRegion(NPP instance, NPRegion invalidRegion)
{
    PluginView* view = pluginViewForInstance(instance);
    if(!view)
       return;
    view->invalidateRegion(invalidRegion);
}

void NPN_ForceRedraw(NPP instance)
{
    PluginView* view = pluginViewForInstance(instance);
    if(!view)
       return;
    view->forceRedraw();
}

NPError NPN_GetValue(NPP instance, NPNVariable variable, void* value)
{
    PluginView* view = pluginViewForInstance(instance);

     if (!view)
         return PluginView::getValueStatic(variable, value);

    return pluginViewForInstance(instance)->getValue(variable, value);
}

NPError NPN_SetValue(NPP instance, NPPVariable variable, void* value)
{
    PluginView* view = pluginViewForInstance(instance);
    if(!view)
       return NPERR_INVALID_INSTANCE_ERROR;
   return view->setValue(variable, value);
}

void* NPN_GetJavaEnv()
{
    // Unsupported
    return 0;
}

void* NPN_GetJavaPeer(NPP instance)
{
    // Unsupported
    return 0;
}

void NPN_PushPopupsEnabledState(NPP instance, NPBool enabled)
{
    PluginView* view = pluginViewForInstance(instance);
    if(!view)
       return;
    view->pushPopupsEnabledState(enabled);
}

void NPN_PopPopupsEnabledState(NPP instance)
{
    PluginView* view = pluginViewForInstance(instance);
    if(!view)
       return;
    view->popPopupsEnabledState();
}

void NPN_PluginThreadAsyncCall(NPP instance, void (*func) (void *), void *userData)
{
    PluginMainThreadScheduler::scheduler().scheduleCall(instance, func, userData);
}

#ifdef PLUGIN_SCHEDULE_TIMER
uint32 NPN_ScheduleTimer(NPP instance, uint32 interval, NPBool repeat,
                         void (*timerFunc)(NPP npp, uint32 timerID))
{
    PluginView* view = pluginViewForInstance(instance);
    if(!view)
       return 0;
    return view->scheduleTimer(instance, interval,
                                                        repeat != 0, timerFunc);
}

void NPN_UnscheduleTimer(NPP instance, uint32 timerID)
{
    PluginView* view = pluginViewForInstance(instance);
    if(!view)
       return;
    view->unscheduleTimer(instance, timerID);
}
#endif


