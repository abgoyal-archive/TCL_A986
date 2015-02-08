
package proguard.evaluation.value;

final class NegatedFloatValue extends SpecificFloatValue
{
    private final FloatValue floatValue;


    /**
     * Creates a new negated float value of the given float value.
     */
    public NegatedFloatValue(FloatValue floatValue)
    {
        this.floatValue = floatValue;
    }


    // Implementations of unary methods of FloatValue.

    public FloatValue negate()
    {
        return floatValue;
    }


    // Implementations for Object.

    public boolean equals(Object object)
    {
        return this == object ||
               super.equals(object) &&
               this.floatValue.equals(((NegatedFloatValue)object).floatValue);
    }


    public int hashCode()
    {
        return super.hashCode() ^
               floatValue.hashCode();
    }


    public String toString()
    {
        return "-"+floatValue;
    }
}