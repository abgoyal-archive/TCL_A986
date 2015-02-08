

#include "PluginObject.h"

#include <stdio.h>

extern "C"
NPError __stdcall NP_Initialize(NPNetscapeFuncs* browserFuncs)
{
    browser = browserFuncs;
    return NPERR_NO_ERROR;
}

extern "C" 
NPError __stdcall NP_GetEntryPoints(NPPluginFuncs* pluginFuncs)
{
    pluginFuncs->version = 11;
    pluginFuncs->size = sizeof(pluginFuncs);
    pluginFuncs->newp = NPP_New;
    pluginFuncs->destroy = NPP_Destroy;
    pluginFuncs->setwindow = NPP_SetWindow;
    pluginFuncs->newstream = NPP_NewStream;
    pluginFuncs->destroystream = NPP_DestroyStream;
    pluginFuncs->asfile = NPP_StreamAsFile;
    pluginFuncs->writeready = NPP_WriteReady;
    pluginFuncs->write = (NPP_WriteProcPtr)NPP_Write;
    pluginFuncs->print = NPP_Print;
    pluginFuncs->event = NPP_HandleEvent;
    pluginFuncs->urlnotify = NPP_URLNotify;
    pluginFuncs->getvalue = NPP_GetValue;
    pluginFuncs->setvalue = NPP_SetValue;
    
    return NPERR_NO_ERROR;
}


extern "C"
NPError __stdcall NP_Shutdown()
{
    return NPERR_NO_ERROR;
}

static void executeScript(const PluginObject* object, const char* script)
{
    NPObject *windowScriptObject;
    browser->getvalue(object->npp, NPNVWindowNPObject, &windowScriptObject);

    NPString npScript;
    npScript.UTF8Characters = script;
    npScript.UTF8Length = strlen(script);

    NPVariant browserResult;
    browser->evaluate(object->npp, windowScriptObject, &npScript, &browserResult);
    browser->releasevariantvalue(&browserResult);
}

NPError NPP_New(NPMIMEType pluginType, NPP instance, uint16 mode, int16 argc, char *argn[], char *argv[], NPSavedData *saved)
{
    if (browser->version >= 14) {
        PluginObject* obj = (PluginObject*)browser->createobject(instance, getPluginClass());
        
        for (int16 i = 0; i < argc; i++) {
            if (_stricmp(argn[i], "onstreamload") == 0 && !obj->onStreamLoad)
                obj->onStreamLoad = _strdup(argv[i]);
            else if (_stricmp(argn[i], "onStreamDestroy") == 0 && !obj->onStreamDestroy)
                obj->onStreamDestroy = _strdup(argv[i]);
            else if (_stricmp(argn[i], "onURLNotify") == 0 && !obj->onURLNotify)
                obj->onURLNotify = _strdup(argv[i]);
            else if (_stricmp(argn[i], "onDestroy") == 0 && !obj->onDestroy)
                obj->onDestroy = _strdup(argv[i]);
            else if (_stricmp(argn[i], "logSrc") == 0) {
                for (int i = 0; i < argc; i++)
                    if (_stricmp(argn[i], "src") == 0)
                        pluginLog(instance, "src: %s", argv[i]);
            } else if (_stricmp(argn[i], "testdocumentopenindestroystream") == 0)
                obj->testDocumentOpenInDestroyStream = TRUE;
              else if (_stricmp(argn[i], "testwindowopen") == 0)
                obj->testWindowOpen = TRUE;
        }
        
        instance->pdata = obj;
    }

    return NPERR_NO_ERROR;
}

NPError NPP_Destroy(NPP instance, NPSavedData **save)
{
    PluginObject *obj = (PluginObject*)instance->pdata;
    if (obj) {
        if (obj->onStreamLoad)
            free(obj->onStreamLoad);

        if (obj->onURLNotify)
            free(obj->onURLNotify);

        if (obj->onStreamDestroy)
            free(obj->onStreamDestroy);

        if (obj->onDestroy) {
            executeScript(obj, obj->onDestroy);
            free(obj->onDestroy);
        }

        if (obj->logDestroy)
            printf("PLUGIN: NPP_Destroy\n");

        browser->releaseobject(&obj->header);
    }
    return NPERR_NO_ERROR;
}

NPError NPP_SetWindow(NPP instance, NPWindow *window)
{
    PluginObject* obj = static_cast<PluginObject*>(instance->pdata);

    if (obj) {
        if (obj->testWindowOpen) {
            testWindowOpen(instance);
            obj->testWindowOpen = FALSE;
        }
    }

    return NPERR_NO_ERROR;
}

NPError NPP_NewStream(NPP instance, NPMIMEType type, NPStream *stream, NPBool seekable, uint16 *stype)
{
    PluginObject* obj = (PluginObject*)instance->pdata;

    if (obj->returnErrorFromNewStream)
        return NPERR_GENERIC_ERROR;

    obj->stream = stream;
    *stype = NP_ASFILEONLY;

    if (obj->onStreamLoad)
        executeScript(obj, obj->onStreamLoad);

    return NPERR_NO_ERROR;
}

NPError NPP_DestroyStream(NPP instance, NPStream *stream, NPReason reason)
{
    PluginObject* obj = (PluginObject*)instance->pdata;

    if (obj->onStreamDestroy)
        executeScript(obj, obj->onStreamDestroy);

    if (obj->testDocumentOpenInDestroyStream) {
        testDocumentOpen(instance);
    }

    return NPERR_NO_ERROR;
}

int32 NPP_WriteReady(NPP instance, NPStream *stream)
{
    return 0;
}

int32 NPP_Write(NPP instance, NPStream *stream, int32 offset, int32 len, void *buffer)
{
    return 0;
}

void NPP_StreamAsFile(NPP instance, NPStream *stream, const char *fname)
{
}

void NPP_Print(NPP instance, NPPrint *platformPrint)
{
}

int16 NPP_HandleEvent(NPP instance, void *event)
{
    PluginObject *obj = (PluginObject*)instance->pdata;
    if (!obj->eventLogging)
        return 0;
    
    // FIXME: Implement this
    return 0;
}

void NPP_URLNotify(NPP instance, const char *url, NPReason reason, void *notifyData)
{
    PluginObject *obj = (PluginObject*)instance->pdata;

    if (obj->onURLNotify)
        executeScript(obj, obj->onURLNotify);
        
    handleCallback(obj, url, reason, notifyData);
}

NPError NPP_GetValue(NPP instance, NPPVariable variable, void *value)
{
    if (variable == NPPVpluginScriptableNPObject) {
        void **v = (void **)value;
        PluginObject *obj = (PluginObject*)instance->pdata;
        // Return value is expected to be retained
        browser->retainobject((NPObject *)obj);
        *v = obj;
        return NPERR_NO_ERROR;
    }
    return NPERR_GENERIC_ERROR;
}

NPError NPP_SetValue(NPP instance, NPNVariable variable, void *value)
{
    return NPERR_GENERIC_ERROR;
}
