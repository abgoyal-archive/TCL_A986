

#if ENABLE(VIDEO)

#include "config.h"
#include "MediaPlayerProxy.h"

#include "Bridge.h"
#include "DocumentLoader.h"
#include "HTMLPlugInElement.h"
#include "HTMLVideoElement.h"
#include "JSDOMBinding.h"
#include "JSPluginElementFunctions.h"
#include "MediaPlayer.h"
#include "Node.h"
#include "PlatformString.h"
#include "PluginView.h"
#include "RenderPartObject.h"
#include "RenderWidget.h"
#include "Widget.h"
#include "c_class.h"
#include "c_instance.h"
#include "c_runtime.h"
#include "npruntime_impl.h"
#include <runtime/Identifier.h>

using namespace JSC;

namespace WebCore {

using namespace Bindings;
using namespace HTMLNames;

WebMediaPlayerProxy::WebMediaPlayerProxy(MediaPlayer* player)
    : m_mediaPlayer(player)
    , m_init(false)
    , m_hasSentResponseToPlugin(false)
{
    if (!m_init)
        initEngine();
}

WebMediaPlayerProxy::~WebMediaPlayerProxy()
{
    m_instance.release();
}

ScriptInstance WebMediaPlayerProxy::pluginInstance()
{
    if (!m_instance) {
        RenderObject* r = element()->renderer();
        if (!r || !r->isWidget())
            return 0;

        Frame* frame = element()->document()->frame();

        RenderWidget* renderWidget = static_cast<RenderWidget*>(element()->renderer());
        if (renderWidget && renderWidget->widget())
            m_instance = frame->script()->createScriptInstanceForWidget(renderWidget->widget());
    }

    return m_instance;
}

void WebMediaPlayerProxy::load(const String& url)
{
    if (!m_init)
        initEngine();
    if (m_init)
        invokeMethod("play");
}

void WebMediaPlayerProxy::initEngine()
{
    HTMLMediaElement* element = static_cast<HTMLMediaElement*>(m_mediaPlayer->mediaPlayerClient());
    String url = element->initialURL();

    if (url.isEmpty())
        return;

    Frame* frame = element->document()->frame();
    Vector<String> paramNames;
    Vector<String> paramValues;
    String serviceType;

    // add all attributes set on the embed object
    if (NamedNodeMap* attributes = element->attributes()) {
        for (unsigned i = 0; i < attributes->length(); ++i) {
            Attribute* it = attributes->attributeItem(i);
            paramNames.append(it->name().localName().string());
            paramValues.append(it->value().string());
        }
    }
    serviceType = "application/x-mplayer2";
    frame->loader()->requestObject(static_cast<RenderPartObject*>(element->renderer()), url, nullAtom, serviceType, paramNames, paramValues);
    m_init = true;

}

HTMLMediaElement* WebMediaPlayerProxy::element()
{
    return static_cast<HTMLMediaElement*>(m_mediaPlayer->mediaPlayerClient());

}

void WebMediaPlayerProxy::invokeMethod(const String& methodName)
{
    Frame* frame = element()->document()->frame();
    RootObject *root = frame->script()->bindingRootObject();
    if (!root)
        return;
    ExecState *exec = root->globalObject()->globalExec();
    Instance* instance = pluginInstance().get();
    if (!instance)
        return;

    instance->begin();
    Class *aClass = instance->getClass();
    Identifier iden(exec, methodName);
    MethodList methodList = aClass->methodsNamed(iden, instance);
    ArgList args;
    instance->invokeMethod(exec, methodList , args);
    instance->end();
}

}

#endif
