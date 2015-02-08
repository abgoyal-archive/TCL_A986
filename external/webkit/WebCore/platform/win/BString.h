

#ifndef BString_h
#define BString_h

#if PLATFORM(CF)
typedef const struct __CFString * CFStringRef;
#endif

typedef wchar_t* BSTR;

namespace JSC {
    class UString;
}

namespace WebCore {

    class AtomicString;
    class KURL;
    class String;

    class BString {
    public:
        BString();
        BString(const wchar_t*);
        BString(const wchar_t*, size_t length);
        BString(const String&);
        BString(const AtomicString&);
        BString(const KURL&);
        BString(const JSC::UString&);
#if PLATFORM(CF)
        BString(CFStringRef);
#endif
        ~BString();

        void adoptBSTR(BSTR);

        BString(const BString&);
        BString& operator=(const BString&);
        BString& operator=(const BSTR&);

        operator BSTR() const { return m_bstr; }

        BSTR release() { BSTR result = m_bstr; m_bstr = 0; return result; }

    private:
        BSTR m_bstr;
    };

    bool operator ==(const BString&, const BString&);
    bool operator !=(const BString&, const BString&);
    bool operator ==(const BString&, BSTR);
    bool operator !=(const BString&, BSTR);
    bool operator ==(BSTR, const BString&);
    bool operator !=(BSTR, const BString&);

}

#endif
