

#include "config.h"
#include "WebKitDLL.h"
#include "WebScriptWorld.h"

#include <JavaScriptCore/APICast.h>
#include <WebCore/JSDOMBinding.h>
#include <WebCore/ScriptController.h>

using namespace WebCore;

typedef HashMap<DOMWrapperWorld*, WebScriptWorld*> WorldMap;
static WorldMap& allWorlds()
{
    static WorldMap& map = *new WorldMap;
    return map;
}

inline WebScriptWorld::WebScriptWorld(PassRefPtr<DOMWrapperWorld> world)
    : m_refCount(0)
    , m_world(world)
{
    ASSERT_ARG(world, m_world);

    ASSERT_ARG(world, !allWorlds().contains(m_world.get()));
    allWorlds().add(m_world.get(), this);

    ++gClassCount;
    gClassNameCount.add("WebScriptWorld");
}

WebScriptWorld::~WebScriptWorld()
{
    ASSERT(allWorlds().contains(m_world.get()));
    allWorlds().remove(m_world.get());

    --gClassCount;
    gClassNameCount.remove("WebScriptWorld");
}

WebScriptWorld* WebScriptWorld::standardWorld()
{
    static WebScriptWorld* standardWorld = createInstance(mainThreadNormalWorld()).releaseRef();
    return standardWorld;
}

COMPtr<WebScriptWorld> WebScriptWorld::createInstance()
{
    return createInstance(ScriptController::createWorld());
}

COMPtr<WebScriptWorld> WebScriptWorld::createInstance(PassRefPtr<DOMWrapperWorld> world)
{
    return new WebScriptWorld(world);
}

COMPtr<WebScriptWorld> WebScriptWorld::findOrCreateWorld(DOMWrapperWorld* world)
{
    if (world == mainThreadNormalWorld())
        return standardWorld();

    if (WebScriptWorld* existingWorld = allWorlds().get(world))
        return existingWorld;

    return createInstance(world);
}

ULONG WebScriptWorld::AddRef()
{
    return ++m_refCount;
}

ULONG WebScriptWorld::Release()
{
    ULONG newRefCount = --m_refCount;
    if (!newRefCount)
        delete this;
    return newRefCount;
}

HRESULT WebScriptWorld::QueryInterface(REFIID riid, void** ppvObject)
{
    if (!ppvObject)
        return E_POINTER;
    *ppvObject = 0;

    if (IsEqualIID(riid, __uuidof(WebScriptWorld)))
        *ppvObject = this;
    else if (IsEqualIID(riid, __uuidof(IWebScriptWorld)))
        *ppvObject = static_cast<IWebScriptWorld*>(this);
    else if (IsEqualIID(riid, IID_IUnknown))
        *ppvObject = static_cast<IUnknown*>(this);
    else
        return E_NOINTERFACE;

    AddRef();
    return S_OK;
}

HRESULT WebScriptWorld::standardWorld(IWebScriptWorld** outWorld)
{
    if (!outWorld)
        return E_POINTER;

    *outWorld = standardWorld();
    (*outWorld)->AddRef();
    return S_OK;
}

HRESULT WebScriptWorld::scriptWorldForGlobalContext(JSGlobalContextRef context, IWebScriptWorld** outWorld)
{
    if (!outWorld)
        return E_POINTER;
    return findOrCreateWorld(currentWorld(toJS(context))).copyRefTo(outWorld);
}
