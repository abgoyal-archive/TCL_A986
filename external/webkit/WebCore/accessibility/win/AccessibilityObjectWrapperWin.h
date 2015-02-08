


#ifndef AccessibilityObjectWrapperWin_h
#define AccessibilityObjectWrapperWin_h

namespace WebCore {

    class AccessibilityObject;
 
    class AccessibilityObjectWrapper : public IUnknown {
    public:
        // IUnknown
        virtual HRESULT STDMETHODCALLTYPE QueryInterface(REFIID riid, void** ppvObject) = 0;        
        virtual ULONG STDMETHODCALLTYPE AddRef(void) = 0;
        virtual ULONG STDMETHODCALLTYPE Release(void) = 0;

        virtual void detach() = 0;
        bool attached() const { return m_object; }
        AccessibilityObject* accessibilityObject() const { return m_object; }

    protected:
        AccessibilityObjectWrapper(AccessibilityObject* obj) : m_object(obj) { }
        AccessibilityObjectWrapper() : m_object(0) { }

        AccessibilityObject* m_object;
    };

} // namespace WebCore

#endif // AccessibilityObjectWrapperWin_h
