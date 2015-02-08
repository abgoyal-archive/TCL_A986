

#ifndef WCDataObject_h
#define WCDataObject_h

#include <wtf/Vector.h>
#include <ShlObj.h>
#include <objidl.h>

namespace WebCore {

class String;

class WCDataObject : public IDataObject {
public:
    void CopyMedium(STGMEDIUM* pMedDest, STGMEDIUM* pMedSrc, FORMATETC* pFmtSrc);

    //IUnknown
    virtual HRESULT STDMETHODCALLTYPE QueryInterface(REFIID riid, void** ppvObject);        
    virtual ULONG STDMETHODCALLTYPE AddRef(void);
    virtual ULONG STDMETHODCALLTYPE Release(void);

    //IDataObject
    virtual HRESULT STDMETHODCALLTYPE GetData(FORMATETC* pformatIn, STGMEDIUM* pmedium);
    virtual HRESULT STDMETHODCALLTYPE GetDataHere(FORMATETC* pformat, STGMEDIUM* pmedium);
    virtual HRESULT STDMETHODCALLTYPE QueryGetData(FORMATETC* pformat);
    virtual HRESULT STDMETHODCALLTYPE GetCanonicalFormatEtc(FORMATETC* pformatectIn,FORMATETC* pformatOut);
    virtual HRESULT STDMETHODCALLTYPE SetData(FORMATETC* pformat, STGMEDIUM*pmedium, BOOL release);
    virtual HRESULT STDMETHODCALLTYPE EnumFormatEtc(DWORD dwDirection, IEnumFORMATETC** ppenumFormatEtc);
    virtual HRESULT STDMETHODCALLTYPE DAdvise(FORMATETC*, DWORD, IAdviseSink*, DWORD*);
    virtual HRESULT STDMETHODCALLTYPE DUnadvise(DWORD);
    virtual HRESULT STDMETHODCALLTYPE EnumDAdvise(IEnumSTATDATA**);

    void clearData(CLIPFORMAT);
    
    static HRESULT createInstance(WCDataObject**);
private:
    WCDataObject();
    virtual ~WCDataObject();
    long m_ref;
    Vector<FORMATETC*> m_formats;
    Vector<STGMEDIUM*> m_medium;
};

}

#endif //!WCDataObject_h
