

#include "config.h"
#include "WebKitDLL.h"
#include "WebElementPropertyBag.h"

#include "MarshallingHelpers.h"
#include "DOMCoreClasses.h"
#include "WebFrame.h"
#pragma warning(push, 0)
#include <WebCore/Document.h>
#include <WebCore/Frame.h>
#include <WebCore/HitTestResult.h>
#include <WebCore/FrameLoader.h>
#include <WebCore/Image.h>
#include <WebCore/KURL.h>
#include <WebCore/RenderObject.h>
#pragma warning(pop)

using namespace WebCore;

// WebElementPropertyBag -----------------------------------------------
WebElementPropertyBag::WebElementPropertyBag(const HitTestResult& result)
    : m_result(new HitTestResult(result))
    , m_refCount(0)
{
    gClassCount++;
    gClassNameCount.add("WebElementPropertyBag");
}

WebElementPropertyBag::~WebElementPropertyBag()
{
    gClassCount--;
    gClassNameCount.remove("WebElementPropertyBag");
}

WebElementPropertyBag* WebElementPropertyBag::createInstance(const HitTestResult& result)
{
    WebElementPropertyBag* instance = new WebElementPropertyBag(result); 
    instance->AddRef();

    return instance;
}

// IUnknown -------------------------------------------------------------------

HRESULT STDMETHODCALLTYPE WebElementPropertyBag::QueryInterface(REFIID riid, void** ppvObject)
{
    *ppvObject = 0;
    if (IsEqualGUID(riid, IID_IUnknown))
        *ppvObject = static_cast<IPropertyBag*>(this);
    else if (IsEqualGUID(riid, IID_IPropertyBag))
        *ppvObject = static_cast<IPropertyBag*>(this);
    else
        return E_NOINTERFACE;

    AddRef();
    return S_OK;
}

ULONG STDMETHODCALLTYPE WebElementPropertyBag::AddRef(void)
{
    return ++m_refCount;
}

ULONG STDMETHODCALLTYPE WebElementPropertyBag::Release(void)
{
    ULONG newRef = --m_refCount;
    if (!newRef)
        delete this;

    return newRef;
}

static bool isEqual(LPCWSTR s1, LPCWSTR s2)
{
    return !wcscmp(s1, s2);
}

static HRESULT convertStringToVariant(VARIANT* pVar, const String& string)
{
    V_VT(pVar) = VT_BSTR;
    V_BSTR(pVar) = SysAllocStringLen(string.characters(), string.length());
    if (string.length() && !V_BSTR(pVar))
        return E_OUTOFMEMORY;

    return S_OK;
}


HRESULT STDMETHODCALLTYPE WebElementPropertyBag::Read(LPCOLESTR pszPropName, VARIANT *pVar, IErrorLog * /*pErrorLog*/)
{
    if (!pszPropName)
        return E_POINTER;

    if (!m_result)
        return E_FAIL;

    BSTR key = (BSTR)pszPropName;
    VariantClear(pVar);
    if (isEqual(WebElementDOMNodeKey, key)) {
        IDOMNode* node = DOMNode::createInstance(m_result->innerNonSharedNode());
        V_VT(pVar) = VT_UNKNOWN;
        V_UNKNOWN(pVar) = node;
        return S_OK;
    } else if (isEqual(WebElementFrameKey, key)) {
        if (!(m_result->innerNonSharedNode() && m_result->innerNonSharedNode()->document()
           && m_result->innerNonSharedNode()->document()->frame()))
            return E_FAIL;
        Frame* coreFrame = m_result->innerNonSharedNode()->document()->frame();
        WebFrame* webFrame = static_cast<WebFrame*>(coreFrame->loader()->client());
        IWebFrame* iWebFrame;
        if (FAILED(webFrame->QueryInterface(IID_IWebFrame, (void**)&iWebFrame)))
            return E_FAIL;
        V_VT(pVar) = VT_UNKNOWN;
        V_UNKNOWN(pVar) = iWebFrame;
        return S_OK;
    } else if (isEqual(WebElementImageAltStringKey, key))
        return convertStringToVariant(pVar, m_result->altDisplayString());
    else if (isEqual(WebElementImageKey, key)) {
        V_VT(pVar) = VT_BYREF;
        V_BYREF(pVar) = m_result->image();
        return S_OK;
    } else if (isEqual(WebElementImageRectKey, key)) {
        V_VT(pVar) = VT_ARRAY;
        IntRect boundingBox = m_result->innerNonSharedNode() && m_result->innerNonSharedNode()->renderer() ?
                                m_result->innerNonSharedNode()->renderer()->absoluteBoundingBoxRect(true) : IntRect();
        V_ARRAY(pVar) = MarshallingHelpers::intRectToSafeArray(boundingBox);
        return S_OK;
    } else if (isEqual(WebElementImageURLKey, key))
        return convertStringToVariant(pVar, m_result->absoluteImageURL().string());
    else if (isEqual(WebElementIsSelectedKey, key)) {
        V_VT(pVar) = VT_BOOL;
        if (m_result->isSelected())
            V_BOOL(pVar) = VARIANT_TRUE;
        else
            V_BOOL(pVar) = VARIANT_FALSE;
        return S_OK;
    } else if (isEqual(WebElementSpellingToolTipKey, key)) {
        TextDirection dir;
        return convertStringToVariant(pVar, m_result->spellingToolTip(dir));
    } else if (isEqual(WebElementTitleKey, key)) {
        TextDirection dir;
        return convertStringToVariant(pVar, m_result->title(dir));
    }
    else if (isEqual(WebElementLinkURLKey, key))
        return convertStringToVariant(pVar, m_result->absoluteLinkURL().string());
    else if (isEqual(WebElementLinkTargetFrameKey, key)) {
        if (!m_result->targetFrame())
            return E_FAIL;
        WebFrame* webFrame = kit(m_result->targetFrame());
        IWebFrame* iWebFrame;
        if (FAILED(webFrame->QueryInterface(IID_IWebFrame, (void**)&iWebFrame)))
            return E_FAIL;
        V_VT(pVar) = VT_UNKNOWN;
        V_UNKNOWN(pVar) = iWebFrame;
        return S_OK;
    } else if (isEqual(WebElementLinkTitleKey, key))
        return convertStringToVariant(pVar, m_result->titleDisplayString());
    else if (isEqual(WebElementLinkLabelKey, key))
        return convertStringToVariant(pVar, m_result->textContent());
    else if (isEqual(WebElementIsContentEditableKey, key)) {
        V_VT(pVar) = VT_BOOL;
        if (m_result->isContentEditable())
            V_BOOL(pVar) = VARIANT_TRUE;
        else
            V_BOOL(pVar) = VARIANT_FALSE;
        return S_OK;
    }

    return E_INVALIDARG;
}

HRESULT STDMETHODCALLTYPE WebElementPropertyBag::Write(LPCOLESTR pszPropName, VARIANT* pVar)
{
    if (!pszPropName || !pVar)
        return E_POINTER;

    return E_FAIL;
}
