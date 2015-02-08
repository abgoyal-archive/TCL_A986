

#ifndef JavaStringJSC_h
#define JavaStringJSC_h

#include "JNIUtility.h"
#include "JavaInstanceJSC.h"
#include <runtime/JSLock.h>


namespace JSC {

namespace Bindings {

class JavaStringImpl {
public:
    ~JavaStringImpl()
    {
        JSLock lock(SilenceAssertionsOnly);
        m_rep = 0;
    }

    void init()
    {
        JSLock lock(SilenceAssertionsOnly);
        m_rep = UString().rep();
    }

    void init(JNIEnv* e, jstring s)
    {
        int size = e->GetStringLength(s);
        const jchar* uc = getUCharactersFromJStringInEnv(e, s);
        {
            JSLock lock(SilenceAssertionsOnly);
            m_rep = UString(reinterpret_cast<const UChar*>(uc), size).rep();
        }
        releaseUCharactersForJStringInEnv(e, s, uc);
    }

    const char* UTF8String() const
    {
        if (!m_utf8String.c_str()) {
            JSLock lock(SilenceAssertionsOnly);
            m_utf8String = UString(m_rep).UTF8String();
        }
        return m_utf8String.c_str();
    }
    const jchar* uchars() const { return (const jchar*)m_rep->data(); }
    int length() const { return m_rep->size(); }
    UString uString() const { return UString(m_rep); }

private:
    RefPtr<UString::Rep> m_rep;
    mutable CString m_utf8String;
};

} // namespace Bindings

} // namespace JSC

#endif // JavaStringJSC_h
