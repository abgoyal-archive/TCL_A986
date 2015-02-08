
package proguard.evaluation.value;

final class NegatedIntegerValue extends SpecificIntegerValue
{
    private final IntegerValue integerValue;


    /**
     * Creates a new negated integer value of the given integer value.
     */
    public NegatedIntegerValue(IntegerValue integerValue)
    {
        this.integerValue = integerValue;
    }


    // Implementations of unary methods of IntegerValue.

    public IntegerValue negate()
    {
        return integerValue;
    }


    // Implementations for Object.

    public boolean equals(Object object)
    {
        return this == object ||
               super.equals(object) &&
               this.integerValue.equals(((NegatedIntegerValue)object).integerValue);
    }


    public int hashCode()
    {
        return super.hashCode() ^
               integerValue.hashCode();
    }


    public String toString()
    {
        return "-"+integerValue;
    }
}