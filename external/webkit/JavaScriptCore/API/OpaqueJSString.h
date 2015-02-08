

#ifndef OpaqueJSString_h
#define OpaqueJSString_h

#include <runtime/UString.h>

namespace JSC {
    class Identifier;
    class JSGlobalData;
}

struct OpaqueJSString : public ThreadSafeShared<OpaqueJSString> {

    static PassRefPtr<OpaqueJSString> create() // null
    {
        return adoptRef(new OpaqueJSString);
    }

    static PassRefPtr<OpaqueJSString> create(const UChar* characters, unsigned length)
    {
        return adoptRef(new OpaqueJSString(characters, length));
    }

    static PassRefPtr<OpaqueJSString> create(const JSC::UString&);

    UChar* characters() { return this ? m_characters : 0; }
    unsigned length() { return this ? m_length : 0; }

    JSC::UString ustring() const;
    JSC::Identifier identifier(JSC::JSGlobalData*) const;

private:
    friend class WTF::ThreadSafeShared<OpaqueJSString>;

    OpaqueJSString()
        : m_characters(0)
        , m_length(0)
    {
    }

    OpaqueJSString(const UChar* characters, unsigned length)
        : m_length(length)
    {
        m_characters = new UChar[length];
        memcpy(m_characters, characters, length * sizeof(UChar));
    }

    ~OpaqueJSString()
    {
        delete[] m_characters;
    }

    UChar* m_characters;
    unsigned m_length;
};

#endif
