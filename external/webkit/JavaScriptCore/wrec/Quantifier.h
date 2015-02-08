

#ifndef Quantifier_h
#define Quantifier_h

#include <wtf/Platform.h>

#if ENABLE(WREC)

#include <wtf/Assertions.h>
#include <limits.h>

namespace JSC { namespace WREC {

    struct Quantifier {
        enum Type {
            None,
            Greedy,
            NonGreedy,
            Error,
        };

        Quantifier(Type type = None, unsigned min = 0, unsigned max = Infinity)
            : type(type)
            , min(min)
            , max(max)
        {
            ASSERT(min <= max);
        }

        Type type;

        unsigned min;
        unsigned max;

        static const unsigned Infinity = UINT_MAX;
    };

} } // namespace JSC::WREC

#endif // ENABLE(WREC)

#endif // Quantifier_h
