

#ifndef WTF_StdLibExtras_h
#define WTF_StdLibExtras_h

#include <wtf/Platform.h>
#include <wtf/Assertions.h>

// Use these to declare and define a static local variable (static T;) so that
//  it is leaked so that its destructors are not called at exit. Using this
//  macro also allows workarounds a compiler bug present in Apple's version of GCC 4.0.1.
#ifndef DEFINE_STATIC_LOCAL
#if COMPILER(GCC) && defined(__APPLE_CC__) && __GNUC__ == 4 && __GNUC_MINOR__ == 0 && __GNUC_PATCHLEVEL__ == 1
#define DEFINE_STATIC_LOCAL(type, name, arguments) \
    static type* name##Ptr = new type arguments; \
    type& name = *name##Ptr
#else
#define DEFINE_STATIC_LOCAL(type, name, arguments) \
    static type& name = *new type arguments
#endif
#endif

// OBJECT_OFFSETOF: Like the C++ offsetof macro, but you can use it with classes.
// The magic number 0x4000 is insignificant. We use it to avoid using NULL, since
// NULL can cause compiler problems, especially in cases of multiple inheritance.
#define OBJECT_OFFSETOF(class, field) (reinterpret_cast<ptrdiff_t>(&(reinterpret_cast<class*>(0x4000)->field)) - 0x4000)

// STRINGIZE: Can convert any value to quoted string, even expandable macros
#define STRINGIZE(exp) #exp
#define STRINGIZE_VALUE_OF(exp) STRINGIZE(exp)

namespace WTF {

    /*
     * C++'s idea of a reinterpret_cast lacks sufficient cojones.
     */
    template<typename TO, typename FROM>
    TO bitwise_cast(FROM from)
    {
        COMPILE_ASSERT(sizeof(TO) == sizeof(FROM), WTF_bitwise_cast_sizeof_casted_types_is_equal);
        union {
            FROM from;
            TO to;
        } u;
        u.from = from;
        return u.to;
    }

    // Returns a count of the number of bits set in 'bits'.
    inline size_t bitCount(unsigned bits)
    {
        bits = bits - ((bits >> 1) & 0x55555555);
        bits = (bits & 0x33333333) + ((bits >> 2) & 0x33333333);
        return (((bits + (bits >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
    }

} // namespace WTF

#endif
