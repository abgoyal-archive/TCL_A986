

#include "config.h"
#include "WebKitDLL.h"
#include "DOMCSSClasses.h"

#pragma warning(push, 0)
#include <WebCore/PlatformString.h>
#pragma warning(pop)

// DOMCSSStyleDeclaration - DOMCSSStyleDeclaration ----------------------------

DOMCSSStyleDeclaration::DOMCSSStyleDeclaration(WebCore::CSSStyleDeclaration* s)
: m_style(0)
{
    if (s)
        s->ref();

    m_style = s;
}

DOMCSSStyleDeclaration::~DOMCSSStyleDeclaration()
{
    if (m_style)
        m_style->deref();
}

IDOMCSSStyleDeclaration* DOMCSSStyleDeclaration::createInstance(WebCore::CSSStyleDeclaration* s)
{
    if (!s)
        return 0;

    HRESULT hr;
    IDOMCSSStyleDeclaration* domStyle = 0;

    DOMCSSStyleDeclaration* newStyle = new DOMCSSStyleDeclaration(s);
    hr = newStyle->QueryInterface(IID_IDOMCSSStyleDeclaration, (void**)&domStyle);

    if (FAILED(hr))
        return 0;

    return domStyle;
}

// DOMCSSStyleDeclaration - IUnknown ------------------------------------------

HRESULT STDMETHODCALLTYPE DOMCSSStyleDeclaration::QueryInterface(REFIID riid, void** ppvObject)
{
    *ppvObject = 0;
    if (IsEqualGUID(riid, IID_IDOMCSSStyleDeclaration))
        *ppvObject = static_cast<IDOMCSSStyleDeclaration*>(this);
    else
        return DOMObject::QueryInterface(riid, ppvObject);

    AddRef();
    return S_OK;
}

// DOMCSSStyleDeclaration - IDOMCSSStyleDeclaration ---------------------------

HRESULT STDMETHODCALLTYPE DOMCSSStyleDeclaration::cssText( 
    /* [retval][out] */ BSTR* /*result*/)
{
    ASSERT_NOT_REACHED();
    return E_NOTIMPL;
}

HRESULT STDMETHODCALLTYPE DOMCSSStyleDeclaration::setCssText( 
    /* [in] */ BSTR cssText)
{
    WebCore::String cssTextString(cssText);
    // FIXME: <rdar://5148045> return DOM exception info
    WebCore::ExceptionCode ec;
    m_style->setCssText(cssTextString, ec);
    return S_OK;
}

HRESULT STDMETHODCALLTYPE DOMCSSStyleDeclaration::getPropertyValue( 
    /* [in] */ BSTR propertyName,
    /* [retval][out] */ BSTR* result)
{
    WebCore::String propertyNameString(propertyName);
    WebCore::String value = m_style->getPropertyValue(propertyNameString);
    *result = SysAllocStringLen(value.characters(), value.length());
    if (value.length() && !*result)
        return E_OUTOFMEMORY;
    return S_OK;
}

HRESULT STDMETHODCALLTYPE DOMCSSStyleDeclaration::getPropertyCSSValue( 
    /* [in] */ BSTR /*propertyName*/,
    /* [retval][out] */ IDOMCSSValue** /*result*/)
{
    ASSERT_NOT_REACHED();
    return E_NOTIMPL;
}

HRESULT STDMETHODCALLTYPE DOMCSSStyleDeclaration::removeProperty( 
    /* [in] */ BSTR /*propertyName*/,
    /* [retval][out] */ BSTR* /*result*/)
{
    ASSERT_NOT_REACHED();
    return E_NOTIMPL;
}

HRESULT STDMETHODCALLTYPE DOMCSSStyleDeclaration::getPropertyPriority( 
    /* [in] */ BSTR /*propertyName*/,
    /* [retval][out] */ BSTR* /*result*/)
{
    ASSERT_NOT_REACHED();
    return E_NOTIMPL;
}

HRESULT STDMETHODCALLTYPE DOMCSSStyleDeclaration::setProperty( 
    /* [in] */ BSTR propertyName,
    /* [in] */ BSTR value,
    /* [in] */ BSTR priority)
{
    WebCore::String propertyNameString(propertyName);
    WebCore::String valueString(value);
    WebCore::String priorityString(priority);
    // FIXME: <rdar://5148045> return DOM exception info
    WebCore::ExceptionCode code;
    m_style->setProperty(propertyNameString, valueString, priorityString, code);
    return S_OK;
}

HRESULT STDMETHODCALLTYPE DOMCSSStyleDeclaration::length( 
    /* [retval][out] */ UINT* /*result*/)
{
    ASSERT_NOT_REACHED();
    return E_NOTIMPL;
}

HRESULT STDMETHODCALLTYPE DOMCSSStyleDeclaration::item( 
    /* [in] */ UINT /*index*/,
    /* [retval][out] */ BSTR* /*result*/)
{
    ASSERT_NOT_REACHED();
    return E_NOTIMPL;
}

HRESULT STDMETHODCALLTYPE DOMCSSStyleDeclaration::parentRule( 
    /* [retval][out] */ IDOMCSSRule** /*result*/)
{
    ASSERT_NOT_REACHED();
    return E_NOTIMPL;
}
