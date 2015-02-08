

#include "config.h"
#include "WebKitCOMAPI.h"

#include "WebKit.h"
#include "WebKitDLL.h"

#include <WebCore/COMPtr.h>

struct CLSIDHash {
    static unsigned hash(const CLSID& clsid)
    {
        RPC_STATUS status;
        return ::UuidHash(const_cast<CLSID*>(&clsid), &status);
    }
    static bool equal(const CLSID& a, const CLSID& b) { return ::IsEqualCLSID(a, b); }
    static const bool safeToCompareToEmptyOrDeleted = true;
};

struct CLSIDHashTraits : WTF::GenericHashTraits<CLSID> {
    static void constructDeletedValue(CLSID& slot) { slot = CLSID_NULL; }
    static bool isDeletedValue(const CLSID& value) { return value == CLSID_NULL; }
};

static COMPtr<IClassFactory> classFactory(const CLSID& clsid)
{
    typedef HashMap<CLSID, COMPtr<IClassFactory>, CLSIDHash, CLSIDHashTraits> FactoryMap;
    static FactoryMap& factories = *new FactoryMap;

    pair<FactoryMap::iterator, bool> result = factories.add(clsid, 0);
    COMPtr<IClassFactory>& factory = result.first->second;
    bool added = result.second;
    if (added && FAILED(DllGetClassObject(clsid, __uuidof(factory), reinterpret_cast<void**>(&factory))))
        factory = 0;

    return factory;
}

HRESULT WebKitCreateInstance(REFCLSID rclsid, IUnknown* pUnkOuter, REFIID riid, void** ppvObject)
{
    COMPtr<IClassFactory> factory = classFactory(rclsid);
    if (!factory)
        return REGDB_E_CLASSNOTREG;

    return factory->CreateInstance(pUnkOuter, riid, ppvObject);
}
