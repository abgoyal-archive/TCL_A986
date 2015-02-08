

#include "config.h"
#include "Collator.h"

#if !USE(ICU_UNICODE) || UCONFIG_NO_COLLATION

namespace WTF {

Collator::Collator(const char*)
{
}

Collator::~Collator()
{
}

void Collator::setOrderLowerFirst(bool)
{
}

std::auto_ptr<Collator> Collator::userDefault()
{
    return std::auto_ptr<Collator>(new Collator(0));
}

// A default implementation for platforms that lack Unicode-aware collation.
Collator::Result Collator::collate(const UChar* lhs, size_t lhsLength, const UChar* rhs, size_t rhsLength) const
{
    int lmin = lhsLength < rhsLength ? lhsLength : rhsLength;
    int l = 0;
    while (l < lmin && *lhs == *rhs) {
        lhs++;
        rhs++;
        l++;
    }

    if (l < lmin)
        return (*lhs > *rhs) ? Greater : Less;

    if (lhsLength == rhsLength)
        return Equal;

    return (lhsLength > rhsLength) ? Greater : Less;
}

}

#endif
