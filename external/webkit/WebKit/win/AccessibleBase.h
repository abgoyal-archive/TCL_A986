

#ifndef AccessibleBase_h
#define AccessibleBase_h

#include <WebCore/AccessibilityObject.h>
#include <WebCore/AccessibilityObjectWrapperWin.h>
#include <WebKit/WebKit.h>
#include <oleacc.h>

class DECLSPEC_UUID("3dbd565b-db22-4d88-8e0e-778bde54524a") AccessibleBase : public IAccessibleComparable, public IServiceProvider, public WebCore::AccessibilityObjectWrapper {
public:
    static AccessibleBase* createInstance(WebCore::AccessibilityObject*);

    // IServiceProvider
    virtual HRESULT STDMETHODCALLTYPE QueryService(REFGUID guidService, REFIID riid, void **ppv);

    // IUnknown
    virtual HRESULT STDMETHODCALLTYPE QueryInterface(REFIID riid, void** ppvObject);
    virtual ULONG STDMETHODCALLTYPE AddRef(void) { return ++m_refCount; }
    virtual ULONG STDMETHODCALLTYPE Release(void);

    // IAccessible
    virtual HRESULT STDMETHODCALLTYPE get_accParent(IDispatch**);
    virtual HRESULT STDMETHODCALLTYPE get_accChildCount(long*);
    virtual HRESULT STDMETHODCALLTYPE get_accChild(VARIANT vChild, IDispatch** ppChild);
    virtual HRESULT STDMETHODCALLTYPE get_accName(VARIANT vChild, BSTR*);
    virtual HRESULT STDMETHODCALLTYPE get_accValue(VARIANT vChild, BSTR*);
    virtual HRESULT STDMETHODCALLTYPE get_accDescription(VARIANT, BSTR*);
    virtual HRESULT STDMETHODCALLTYPE get_accRole(VARIANT vChild, VARIANT* pvRole);
    virtual HRESULT STDMETHODCALLTYPE get_accState(VARIANT vChild, VARIANT* pvState);
    virtual HRESULT STDMETHODCALLTYPE get_accHelp(VARIANT vChild, BSTR* helpText);
    virtual HRESULT STDMETHODCALLTYPE get_accKeyboardShortcut(VARIANT vChild, BSTR*);
    virtual HRESULT STDMETHODCALLTYPE get_accFocus(VARIANT* pvFocusedChild);
    virtual HRESULT STDMETHODCALLTYPE get_accSelection(VARIANT* pvSelectedChild);
    virtual HRESULT STDMETHODCALLTYPE get_accDefaultAction(VARIANT vChild, BSTR* actionDescription);
    virtual HRESULT STDMETHODCALLTYPE accSelect(long selectionFlags, VARIANT vChild);
    virtual HRESULT STDMETHODCALLTYPE accLocation(long* left, long* top, long* width, long* height, VARIANT vChild);
    virtual HRESULT STDMETHODCALLTYPE accNavigate(long direction, VARIANT vFromChild, VARIANT* pvNavigatedTo);
    virtual HRESULT STDMETHODCALLTYPE accHitTest(long x, long y, VARIANT* pvChildAtPoint);
    virtual HRESULT STDMETHODCALLTYPE accDoDefaultAction(VARIANT vChild);

    // IAccessible - Not to be implemented.
    virtual HRESULT STDMETHODCALLTYPE put_accName(VARIANT, BSTR) { return E_NOTIMPL; }
    virtual HRESULT STDMETHODCALLTYPE put_accValue(VARIANT, BSTR) { return E_NOTIMPL; }
    virtual HRESULT STDMETHODCALLTYPE get_accHelpTopic(BSTR* helpFile, VARIANT, long* topicID)
    {
        *helpFile = 0;
        *topicID = 0;
        return E_NOTIMPL;
    }

    // IDispatch - Not to be implemented.
    virtual HRESULT STDMETHODCALLTYPE GetTypeInfoCount(UINT* count)
    {
        *count = 0;
        return E_NOTIMPL;
    }
    virtual HRESULT STDMETHODCALLTYPE GetTypeInfo(UINT, LCID, ITypeInfo** ppTInfo)
    {
        *ppTInfo = 0;
        return E_NOTIMPL;
    }
    virtual HRESULT STDMETHODCALLTYPE GetIDsOfNames(REFIID, LPOLESTR*, UINT, LCID, DISPID*) { return E_NOTIMPL; }
    virtual HRESULT STDMETHODCALLTYPE Invoke(DISPID, REFIID, LCID, WORD, DISPPARAMS*, VARIANT*, EXCEPINFO*, UINT*) { return E_NOTIMPL; }

    // WebCore::AccessiblityObjectWrapper
    virtual void detach() {
        ASSERT(m_object);
        m_object = 0;
    }

    // IAccessibleComparable
    virtual HRESULT STDMETHODCALLTYPE isSameObject(IAccessibleComparable* other, BOOL* result);

protected:
    AccessibleBase(WebCore::AccessibilityObject*);
    virtual ~AccessibleBase();

    virtual WebCore::String name() const;
    virtual WebCore::String value() const;
    virtual long role() const;

    HRESULT getAccessibilityObjectForChild(VARIANT vChild, WebCore::AccessibilityObject*&) const;

    static AccessibleBase* wrapper(WebCore::AccessibilityObject*);

    int m_refCount;

private:
    AccessibleBase() { }
};

#endif // AccessibleBase_h

