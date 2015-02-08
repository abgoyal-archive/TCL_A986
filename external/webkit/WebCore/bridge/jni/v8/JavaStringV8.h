

#ifndef JavaStringV8_h
#define JavaStringV8_h

#include "CString.h"
#include "JNIUtility.h"


namespace JSC {

namespace Bindings {

class JavaStringImpl {
public:
    void init() {}

    void init(JNIEnv* e, jstring s)
    {
        int size = e->GetStringLength(s);
        const char* cs = getCharactersFromJStringInEnv(e, s);
        m_utf8String = WebCore::CString(cs, size);
        releaseCharactersForJStringInEnv(e, s, cs);
    }

    const char* UTF8String() const { return m_utf8String.data(); }
    const jchar* uchars() const { return 0; } // Not implemented
    int length() const { return m_utf8String.length(); }

private:
    WebCore::CString m_utf8String;
};

} // namespace Bindings

} // namespace JSC

#endif // JavaStringV8_h
