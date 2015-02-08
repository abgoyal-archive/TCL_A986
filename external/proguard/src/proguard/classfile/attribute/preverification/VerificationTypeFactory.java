
package proguard.classfile.attribute.preverification;

public class VerificationTypeFactory
{
    // Shared copies of Type objects, to avoid creating a lot of objects.
    static final IntegerType           INTEGER_TYPE            = new IntegerType();
    static final LongType              LONG_TYPE               = new LongType();
    static final FloatType             FLOAT_TYPE              = new FloatType();
    static final DoubleType            DOUBLE_TYPE             = new DoubleType();
    static final TopType               TOP_TYPE                = new TopType();
    static final NullType              NULL_TYPE               = new NullType();
    static final UninitializedThisType UNINITIALIZED_THIS_TYPE = new UninitializedThisType();


    /**
     * Creates a new IntegerType.
     */
    public static IntegerType createIntegerType()
    {
        return INTEGER_TYPE;
    }

    /**
     * Creates a new LongType.
     */
    public static LongType createLongType()
    {
        return LONG_TYPE;
    }

    /**
     * Creates a new FloatType.
     */
    public static FloatType createFloatType()
    {
        return FLOAT_TYPE;
    }

    /**
     * Creates a new DoubleType.
     */
    public static DoubleType createDoubleType()
    {
        return DOUBLE_TYPE;
    }

    /**
     * Creates a new TopType.
     */
    public static TopType createTopType()
    {
        return TOP_TYPE;
    }

    /**
     * Creates a new NullType.
     */
    public static NullType createNullType()
    {
        return NULL_TYPE;
    }

    /**
     * Creates a new UninitializedThisType.
     */
    public static UninitializedThisType createUninitializedThisType()
    {
        return UNINITIALIZED_THIS_TYPE;
    }

    /**
     * Creates a new UninitializedType for an instance that was created at
     * the given offset.
     */
    public static UninitializedType createUninitializedType(int newInstructionOffset)
    {
        return new UninitializedType(newInstructionOffset);
    }

    /**
     * Creates a new ObjectType of the given type.
     */
    public static ObjectType createObjectType(int classIndex)
    {
        return new ObjectType(classIndex);
    }
}
