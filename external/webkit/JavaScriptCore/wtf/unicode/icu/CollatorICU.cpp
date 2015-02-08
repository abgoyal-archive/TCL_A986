

#include "config.h"
#include "Collator.h"

#if USE(ICU_UNICODE) && !UCONFIG_NO_COLLATION

#include "Assertions.h"
#include "Threading.h"
#include <unicode/ucol.h>
#include <string.h>

#if OS(DARWIN)
#include "RetainPtr.h"
#include <CoreFoundation/CoreFoundation.h>
#endif

namespace WTF {

static UCollator* cachedCollator;
static Mutex& cachedCollatorMutex()
{
    AtomicallyInitializedStatic(Mutex&, mutex = *new Mutex);
    return mutex;
}

Collator::Collator(const char* locale)
    : m_collator(0)
    , m_locale(locale ? strdup(locale) : 0)
    , m_lowerFirst(false)
{
}

std::auto_ptr<Collator> Collator::userDefault()
{
#if OS(DARWIN) && PLATFORM(CF)
    // Mac OS X doesn't set UNIX locale to match user-selected one, so ICU default doesn't work.
#if !defined(BUILDING_ON_TIGER) && !defined(BUILDING_ON_LEOPARD) && !OS(IPHONE_OS)
    RetainPtr<CFLocaleRef> currentLocale(AdoptCF, CFLocaleCopyCurrent());
    CFStringRef collationOrder = (CFStringRef)CFLocaleGetValue(currentLocale.get(), kCFLocaleCollatorIdentifier);
#else
    RetainPtr<CFStringRef> collationOrderRetainer(AdoptCF, (CFStringRef)CFPreferencesCopyValue(CFSTR("AppleCollationOrder"), kCFPreferencesAnyApplication, kCFPreferencesCurrentUser, kCFPreferencesAnyHost));
    CFStringRef collationOrder = collationOrderRetainer.get();
#endif
    char buf[256];
    if (collationOrder) {
        CFStringGetCString(collationOrder, buf, sizeof(buf), kCFStringEncodingASCII);
        return std::auto_ptr<Collator>(new Collator(buf));
    } else
        return std::auto_ptr<Collator>(new Collator(""));
#else
    return std::auto_ptr<Collator>(new Collator(0));
#endif
}

Collator::~Collator()
{
    releaseCollator();
    free(m_locale);
}

void Collator::setOrderLowerFirst(bool lowerFirst)
{
    m_lowerFirst = lowerFirst;
}

Collator::Result Collator::collate(const UChar* lhs, size_t lhsLength, const UChar* rhs, size_t rhsLength) const
{
    if (!m_collator)
        createCollator();

    return static_cast<Result>(ucol_strcoll(m_collator, lhs, lhsLength, rhs, rhsLength));
}

void Collator::createCollator() const
{
    ASSERT(!m_collator);
    UErrorCode status = U_ZERO_ERROR;

    {
        Locker<Mutex> lock(cachedCollatorMutex());
        if (cachedCollator) {
            const char* cachedCollatorLocale = ucol_getLocaleByType(cachedCollator, ULOC_REQUESTED_LOCALE, &status);
            ASSERT(U_SUCCESS(status));
            ASSERT(cachedCollatorLocale);

            UColAttributeValue cachedCollatorLowerFirst = ucol_getAttribute(cachedCollator, UCOL_CASE_FIRST, &status);
            ASSERT(U_SUCCESS(status));

            // FIXME: default locale is never matched, because ucol_getLocaleByType returns the actual one used, not 0.
            if (m_locale && 0 == strcmp(cachedCollatorLocale, m_locale)
                && ((UCOL_LOWER_FIRST == cachedCollatorLowerFirst && m_lowerFirst) || (UCOL_UPPER_FIRST == cachedCollatorLowerFirst && !m_lowerFirst))) {
                m_collator = cachedCollator;
                cachedCollator = 0;
                return;
            }
        }
    }

    m_collator = ucol_open(m_locale, &status);
    if (U_FAILURE(status)) {
        status = U_ZERO_ERROR;
        m_collator = ucol_open("", &status); // Fallback to Unicode Collation Algorithm.
    }
    ASSERT(U_SUCCESS(status));

    ucol_setAttribute(m_collator, UCOL_CASE_FIRST, m_lowerFirst ? UCOL_LOWER_FIRST : UCOL_UPPER_FIRST, &status);
    ASSERT(U_SUCCESS(status));
}

void Collator::releaseCollator()
{
    {
        Locker<Mutex> lock(cachedCollatorMutex());
        if (cachedCollator)
            ucol_close(cachedCollator);
        cachedCollator = m_collator;
        m_collator  = 0;
    }
}

}

#endif
