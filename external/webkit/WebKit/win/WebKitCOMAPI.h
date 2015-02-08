

#ifndef WebKitCOMAPI_h
#define WebKitCOMAPI_h

extern "C" {

// Callers should use this API instead of CoCreateInstance to instantiate WebKit COM classes.
// This bypasses CoCreateInstance entirely, so registry keys and isolated COM manifests aren't needed.
HRESULT WebKitCreateInstance(REFCLSID, IUnknown* pUnkOuter, REFIID, void** ppvObject);

}

#endif // !defined(WebKitCOMAPI_h)
