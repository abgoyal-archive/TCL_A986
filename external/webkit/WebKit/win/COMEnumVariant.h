

#ifndef COMEnumVariant_h
#define COMEnumVariant_h

#define NOMINMAX
#include <unknwn.h>

#include <wtf/Noncopyable.h>

#include "COMVariantSetter.h"

template<typename ContainerType>
class COMEnumVariant : public IEnumVARIANT, public Noncopyable {
public:
    static COMEnumVariant* adopt(ContainerType&);
    static COMEnumVariant* createInstance(const ContainerType&);

    // IUnknown
    virtual HRESULT STDMETHODCALLTYPE QueryInterface(REFIID riid, void** ppvObject);
    virtual ULONG STDMETHODCALLTYPE AddRef();
    virtual ULONG STDMETHODCALLTYPE Release();

    // IEnumVARIANT
    virtual HRESULT STDMETHODCALLTYPE Next(ULONG celt, VARIANT* rgVar, ULONG* pCeltFetched);
    virtual HRESULT STDMETHODCALLTYPE Skip(ULONG celt);
    virtual HRESULT STDMETHODCALLTYPE Reset();
    virtual HRESULT STDMETHODCALLTYPE Clone(IEnumVARIANT** ppEnum);

private:
    COMEnumVariant()
        : m_refCount(0)
    {
    }

    COMEnumVariant(const ContainerType& container)
        : m_refCount(0)
        , m_container(container)       
        , m_currentPos(m_container.begin())
    {
    }

    ~COMEnumVariant() {}

    ULONG m_refCount;

    ContainerType m_container;
    typename ContainerType::const_iterator m_currentPos;
};

// COMEnumVariant ------------------------------------------------------------------
template<typename ContainerType>
COMEnumVariant<typename ContainerType>* COMEnumVariant<ContainerType>::adopt(ContainerType& container) 
{
    COMEnumVariant* instance = new COMEnumVariant;
    instance->m_container.swap(container);
    instance->m_currentPos = instance->m_container.begin();
    instance->AddRef();
    return instance;
}

template<typename ContainerType>
COMEnumVariant<typename ContainerType>* COMEnumVariant<ContainerType>::createInstance(const ContainerType& container)
{
    COMEnumVariant* instance = new COMEnumVariant(container);
    instance->AddRef();
    return instance;
}

// IUnknown ------------------------------------------------------------------------
template<typename ContainerType>
HRESULT STDMETHODCALLTYPE COMEnumVariant<ContainerType>::QueryInterface(REFIID riid, void** ppvObject)
{
    *ppvObject = 0;
    if (IsEqualGUID(riid, IID_IUnknown))
        *ppvObject = static_cast<COMEnumVariant*>(this);
    else if (IsEqualGUID(riid, IID_IEnumVARIANT))
        *ppvObject = static_cast<COMEnumVariant*>(this);
    else
        return E_NOINTERFACE;

    AddRef();
    return S_OK;
}

template<typename ContainerType>
ULONG STDMETHODCALLTYPE COMEnumVariant<ContainerType>::AddRef()
{
    return ++m_refCount;
}

template<typename ContainerType>
ULONG STDMETHODCALLTYPE COMEnumVariant<ContainerType>::Release()
{
    ULONG newRef = --m_refCount;
    if (!newRef)
        delete this;

    return newRef;
}

// IEnumVARIANT --------------------------------------------------------------------
template<typename ContainerType>
HRESULT STDMETHODCALLTYPE COMEnumVariant<ContainerType>::Next(ULONG celt, VARIANT* rgVar, ULONG* pCeltFetched)
{
    if (pCeltFetched)
        *pCeltFetched = 0;
    if (!rgVar)
        return E_POINTER;
    for (unsigned i = 0 ; i < celt; i++)
        VariantInit(&rgVar[i]);

    for (unsigned i = 0; i < celt; i++) {
        if (m_currentPos == m_container.end())
            return S_FALSE;

        COMVariantSetter<ContainerType::ValueType>::setVariant(&rgVar[i], *m_currentPos);
        ++m_currentPos;
        if (pCeltFetched)
            (*pCeltFetched)++;
    }

    return S_OK;
}

template<typename ContainerType>
HRESULT STDMETHODCALLTYPE COMEnumVariant<ContainerType>::Skip(ULONG celt) 
{
    for (unsigned i = 0; i < celt; i++) {
        if (m_currentPos == m_container.end())
            return S_FALSE;

        ++m_currentPos;
    }
    return S_OK;
}
    
template<typename ContainerType>
HRESULT STDMETHODCALLTYPE COMEnumVariant<ContainerType>::Reset() 
{
    m_currentPos = m_container.begin();
    return S_OK;
}
    
template<typename ContainerType>
HRESULT STDMETHODCALLTYPE COMEnumVariant<ContainerType>::Clone(IEnumVARIANT** ppEnum)
{
    if (!ppEnum)
        return E_POINTER;

    *ppEnum = 0;
    return E_NOTIMPL;
}

#endif
