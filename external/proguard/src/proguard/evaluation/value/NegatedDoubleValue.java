
package proguard.evaluation.value;

final class NegatedDoubleValue extends SpecificDoubleValue
{
    private final DoubleValue doubleValue;


    /**
     * Creates a new negated double value of the given double value.
     */
    public NegatedDoubleValue(DoubleValue doubleValue)
    {
        this.doubleValue = doubleValue;
    }


    // Implementations of unary methods of DoubleValue.

    public DoubleValue negate()
    {
        return doubleValue;
    }


    // Implementations for Object.

    public boolean equals(Object object)
    {
        return this == object ||
               super.equals(object) &&
               this.doubleValue.equals(((NegatedDoubleValue)object).doubleValue);
    }


    public int hashCode()
    {
        return super.hashCode() ^
               doubleValue.hashCode();
    }


    public String toString()
    {
        return "-"+doubleValue;
    }
}