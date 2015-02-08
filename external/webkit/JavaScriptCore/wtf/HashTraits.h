

#ifndef WTF_HashTraits_h
#define WTF_HashTraits_h

#include "HashFunctions.h"
#include "TypeTraits.h"
#include <utility>
#include <limits>

namespace WTF {

    using std::pair;
    using std::make_pair;

    template<typename T> struct HashTraits;

    template<bool isInteger, typename T> struct GenericHashTraitsBase;

    template<typename T> struct GenericHashTraitsBase<false, T> {
        static const bool emptyValueIsZero = false;
        static const bool needsDestruction = true;
    };

    // Default integer traits disallow both 0 and -1 as keys (max value instead of -1 for unsigned).
    template<typename T> struct GenericHashTraitsBase<true, T> {
        static const bool emptyValueIsZero = true;
        static const bool needsDestruction = false;
        static void constructDeletedValue(T& slot) { slot = static_cast<T>(-1); }
        static bool isDeletedValue(T value) { return value == static_cast<T>(-1); }
    };

    template<typename T> struct GenericHashTraits : GenericHashTraitsBase<IsInteger<T>::value, T> {
        typedef T TraitType;
        static T emptyValue() { return T(); }
    };

    template<typename T> struct HashTraits : GenericHashTraits<T> { };

    template<typename T> struct FloatHashTraits : GenericHashTraits<T> {
        static const bool needsDestruction = false;
        static T emptyValue() { return std::numeric_limits<T>::infinity(); }
        static void constructDeletedValue(T& slot) { slot = -std::numeric_limits<T>::infinity(); }
        static bool isDeletedValue(T value) { return value == -std::numeric_limits<T>::infinity(); }
    };

    template<> struct HashTraits<float> : FloatHashTraits<float> { };
    template<> struct HashTraits<double> : FloatHashTraits<double> { };

    // Default unsigned traits disallow both 0 and max as keys -- use these traits to allow zero and disallow max - 1.
    template<typename T> struct UnsignedWithZeroKeyHashTraits : GenericHashTraits<T> {
        static const bool emptyValueIsZero = false;
        static const bool needsDestruction = false;
        static T emptyValue() { return std::numeric_limits<T>::max(); }
        static void constructDeletedValue(T& slot) { slot = std::numeric_limits<T>::max() - 1; }
        static bool isDeletedValue(T value) { return value == std::numeric_limits<T>::max() - 1; }
    };

    template<typename P> struct HashTraits<P*> : GenericHashTraits<P*> {
        static const bool emptyValueIsZero = true;
        static const bool needsDestruction = false;
        static void constructDeletedValue(P*& slot) { slot = reinterpret_cast<P*>(-1); }
        static bool isDeletedValue(P* value) { return value == reinterpret_cast<P*>(-1); }
    };

    template<typename P> struct HashTraits<RefPtr<P> > : GenericHashTraits<RefPtr<P> > {
        static const bool emptyValueIsZero = true;
        static void constructDeletedValue(RefPtr<P>& slot) { new (&slot) RefPtr<P>(HashTableDeletedValue); }
        static bool isDeletedValue(const RefPtr<P>& value) { return value.isHashTableDeletedValue(); }
    };

    // special traits for pairs, helpful for their use in HashMap implementation

    template<typename FirstTraitsArg, typename SecondTraitsArg>
    struct PairHashTraits : GenericHashTraits<pair<typename FirstTraitsArg::TraitType, typename SecondTraitsArg::TraitType> > {
        typedef FirstTraitsArg FirstTraits;
        typedef SecondTraitsArg SecondTraits;
        typedef pair<typename FirstTraits::TraitType, typename SecondTraits::TraitType> TraitType;

        static const bool emptyValueIsZero = FirstTraits::emptyValueIsZero && SecondTraits::emptyValueIsZero;
        static TraitType emptyValue() { return make_pair(FirstTraits::emptyValue(), SecondTraits::emptyValue()); }

        static const bool needsDestruction = FirstTraits::needsDestruction || SecondTraits::needsDestruction;

        static void constructDeletedValue(TraitType& slot) { FirstTraits::constructDeletedValue(slot.first); }
        static bool isDeletedValue(const TraitType& value) { return FirstTraits::isDeletedValue(value.first); }
    };

    template<typename First, typename Second>
    struct HashTraits<pair<First, Second> > : public PairHashTraits<HashTraits<First>, HashTraits<Second> > { };

} // namespace WTF

using WTF::HashTraits;
using WTF::PairHashTraits;

#endif // WTF_HashTraits_h
