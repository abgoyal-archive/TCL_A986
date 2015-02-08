

#ifndef JSType_h
#define JSType_h

namespace JSC {

    /**
     * Primitive types
     */
    enum JSType {
        UnspecifiedType   = 0,
        UndefinedType     = 1,
        BooleanType       = 2,
        NumberType        = 3,
        NullType          = 4,
        StringType        = 5,
        // The CompoundType value must come before any JSType that may have children
        CompoundType      = 6,
        ObjectType        = 7,
        GetterSetterType  = 8
    };

} // namespace JSC

#endif
