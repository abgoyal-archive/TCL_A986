
package proguard.evaluation.value;

final class NegatedLongValue extends SpecificLongValue
{
    private final LongValue longValue;


    /**
     * Creates a new negated long value of the given long value.
     */
    public NegatedLongValue(LongValue longValue)
    {
        this.longValue = longValue;
    }


    // Implementations of unary methods of LongValue.

    public LongValue negate()
    {
        return longValue;
    }


    // Implementations for Object.

    public boolean equals(Object object)
    {
        return this == object ||
               super.equals(object) &&
               this.longValue.equals(((NegatedLongValue)object).longValue);
    }


    public int hashCode()
    {
        return super.hashCode() ^
               longValue.hashCode();
    }


    public String toString()
    {
        return "-"+longValue;
    }
}