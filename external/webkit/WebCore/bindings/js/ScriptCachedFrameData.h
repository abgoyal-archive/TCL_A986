

#ifndef ScriptCachedFrameData_h
#define ScriptCachedFrameData_h

#include <runtime/Protect.h>

namespace WebCore {
    class Frame;
    class JSDOMWindow;
    class DOMWindow;
    class DOMWrapperWorld;

    class ScriptCachedFrameData  : public Noncopyable {
        typedef HashMap< RefPtr<DOMWrapperWorld>, JSC::ProtectedPtr<JSDOMWindow> > JSDOMWindowSet;

    public:
        ScriptCachedFrameData(Frame*);
        ~ScriptCachedFrameData();

        void restore(Frame*);
        void clear();
        DOMWindow* domWindow() const;

    private:
        JSDOMWindowSet m_windows;
        DOMWindow* m_domWindow;
    };

} // namespace WebCore

#endif // ScriptCachedFrameData_h
