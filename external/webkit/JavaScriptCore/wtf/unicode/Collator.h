

#ifndef WTF_Collator_h
#define WTF_Collator_h

#include <memory>
#include <wtf/Noncopyable.h>
#include <wtf/unicode/Unicode.h>

#if USE(ICU_UNICODE) && !UCONFIG_NO_COLLATION
struct UCollator;
#endif

namespace WTF {

    class Collator : public Noncopyable {
    public:
        enum Result { Equal = 0, Greater = 1, Less = -1 };

        Collator(const char* locale); // Parsing is lenient; e.g. language identifiers (such as "en-US") are accepted, too.
        ~Collator();
        void setOrderLowerFirst(bool);

        static std::auto_ptr<Collator> userDefault();

        Result collate(const ::UChar*, size_t, const ::UChar*, size_t) const;

    private:
#if USE(ICU_UNICODE) && !UCONFIG_NO_COLLATION
        void createCollator() const;
        void releaseCollator();
        mutable UCollator* m_collator;
#endif
        char* m_locale;
        bool m_lowerFirst;
    };
}

using WTF::Collator;

#endif
